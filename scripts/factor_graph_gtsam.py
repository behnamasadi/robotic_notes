# import gtsam
# import numpy as np
# import math

# # Create an empty nonlinear factor graph
# graph = gtsam.NonlinearFactorGraph()

# # Add a prior on pose x1 (key 1)
# prior_mean = gtsam.Pose2(0.0, 0.0, 0.0)  # (x, y, theta)
# prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))
# graph.add(gtsam.PriorFactorPose2(1, prior_mean, prior_noise))

# # Odometry measurement: move forward 2m
# odometry = gtsam.Pose2(2.0, 0.0, 0.0)
# odometry_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))

# # Add two odometry (Between) factors
# graph.add(gtsam.BetweenFactorPose2(1, 2, odometry, odometry_noise))
# graph.add(gtsam.BetweenFactorPose2(2, 3, odometry, odometry_noise))

# print("Factor Graph:")
# graph.print()


import gtsam
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import pydot
import tempfile
import os
import argparse

# ----------------------------- Load g2o File -----------------------------


def load_g2o_file(g2o_file):
    """
    Load a g2o file and return the graph and initial estimates.
    """
    print(f"Loading g2o file: {g2o_file}")
    graph, initial = gtsam.readG2o(g2o_file, is3D=False)
    print(f"Loaded {graph.size()} factors and {initial.size()} variables")
    return graph, initial


# ----------------------------- Setup -----------------------------
# Parse command line arguments
parser = argparse.ArgumentParser(
    description="Load and visualize a g2o file using GTSAM iSAM2",
    formatter_class=argparse.RawDescriptionHelpFormatter
)
parser.add_argument(
    "--input", "-i",
    type=str,
    default="/home/behnam/workspace/robotic_notes/data/slam/input_INTEL_g2o.g2o",
    help="Path to the g2o file"
)
parser.add_argument(
    "--incremental",
    action="store_true",
    help="Process factors incrementally (slower but shows incremental updates)"
)
args = parser.parse_args()

# Check if file exists
if not os.path.exists(args.input):
    print(f"Error: File not found: {args.input}")
    exit(1)

# Load g2o file
full_graph, initial_estimate = load_g2o_file(args.input)

# Set up iSAM2
params = gtsam.ISAM2Params()
params.setRelinearizeThreshold(0.1)
params.relinearizeSkip = 1
isam = gtsam.ISAM2(params)

# Add a prior on the first pose to anchor the solution
prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01]))
prior_graph = gtsam.NonlinearFactorGraph()
if initial_estimate.exists(0):
    first_pose = initial_estimate.atPose2(0)
    prior_graph.add(gtsam.PriorFactorPose2(0, first_pose, prior_noise))
else:
    # If pose 0 doesn't exist, use identity
    prior_graph.add(gtsam.PriorFactorPose2(0, gtsam.Pose2(), prior_noise))

# Storage for animation
history = []  # List of (result, affected_cliques) tuples

# ----------------------------- Incremental Solve -----------------------------
if args.incremental:
    # Process factors incrementally (for visualization)
    print("\nProcessing factors incrementally...")

    # Start with prior
    isam.update(prior_graph, gtsam.Values())
    result = isam.calculateEstimate()
    affected = set()
    result_copy = gtsam.Values(result)
    history.append((result_copy, affected))

    # Process factors in batches for visualization
    batch_size = max(1, full_graph.size() // 20)  # 20 steps for animation

    for i in range(0, full_graph.size(), batch_size):
        batch_graph = gtsam.NonlinearFactorGraph()
        batch_initial = gtsam.Values()

        # Add factors in batch
        for j in range(i, min(i + batch_size, full_graph.size())):
            factor = full_graph.at(j)
            batch_graph.add(factor)

            # Extract keys from factor and add initial estimates if needed
            keys = factor.keys()
            for key in keys:
                if not result.exists(key) and initial_estimate.exists(key):
                    batch_initial.insert(key, initial_estimate.atPose2(key))

        if batch_graph.size() > 0:
            isam.update(batch_graph, batch_initial)
            result = isam.calculateEstimate()
            affected = set()
            result_copy = gtsam.Values(result)
            history.append((result_copy, affected))
            print(
                f"Processed {min(i + batch_size, full_graph.size())}/{full_graph.size()} factors")
else:
    # Process all at once (faster)
    print("\nProcessing all factors at once...")
    # Add prior to the graph
    for i in range(prior_graph.size()):
        full_graph.add(prior_graph.at(i))

    isam.update(full_graph, initial_estimate)
    result = isam.calculateEstimate()
    affected = set()
    result_copy = gtsam.Values(result)
    history.append((result_copy, affected))
    print("Optimization completed!")

print(f"Final error: {full_graph.error(result):.6f}")

# ----------------------------- Visualization -----------------------------
# Get final result to determine plot limits
final_result = history[-1][0] if history else result

# Extract keys - try different methods
try:
    keys_list = list(final_result.keys())
except:
    # Alternative: iterate until we can't find a key
    keys_list = []
    i = 0
    while final_result.exists(i):
        keys_list.append(i)
        i += 1

if len(keys_list) == 0:
    print("Error: No poses found in the graph")
    print(f"Values size: {final_result.size()}")
    exit(1)

print(f"Found {len(keys_list)} poses in the graph")

# Calculate trajectory bounds
x_coords = []
y_coords = []
for key in keys_list:
    try:
        pose = final_result.atPose2(key)
        x_coords.append(pose.x())
        y_coords.append(pose.y())
    except:
        continue

if len(x_coords) == 0:
    print("Error: Could not extract any Pose2 values")
    exit(1)

x_min, x_max = min(x_coords), max(x_coords)
y_min, y_max = min(y_coords), max(y_coords)
x_margin = (x_max - x_min) * 0.1 if (x_max - x_min) > 0 else 1.0
y_margin = (y_max - y_min) * 0.1 if (y_max - y_min) > 0 else 1.0

fig, (ax_traj, ax_tree) = plt.subplots(1, 2, figsize=(14, 7))
ax_traj.set_xlim(x_min - x_margin, x_max + x_margin)
ax_traj.set_ylim(y_min - y_margin, y_max + y_margin)
ax_traj.set_aspect('equal')
ax_traj.grid(True)
ax_traj.set_title("Trajectory Estimate")
ax_traj.set_xlabel("X (m)")
ax_traj.set_ylabel("Y (m)")

line_est, = ax_traj.plot([], [], 'b-', linewidth=2, label="Current estimate")
points_est, = ax_traj.plot([], [], 'bo')
ax_traj.legend()

# Bayes tree image placeholder
tree_img = ax_tree.imshow(np.zeros((100, 100, 3)), aspect='equal')
ax_tree.axis('off')
ax_tree.set_title("Bayes Tree (Red = Updated Cliques)")


def update(frame):
    result, affected = history[frame]

    # Trajectory - extract keys
    try:
        keys_list = list(result.keys())
    except:
        keys_list = []
        i = 0
        while result.exists(i):
            keys_list.append(i)
            i += 1

    # Extract x, y coordinates
    x = []
    y = []
    for key in keys_list:
        try:
            pose = result.atPose2(key)
            x.append(pose.x())
            y.append(pose.y())
        except:
            continue

    if len(x) > 0:
        line_est.set_data(x, y)
        points_est.set_data(x, y)

    # Bayes tree - try to visualize, but handle failures gracefully
    try:
        # Try using isam directly (it inherits from BayesTree)
        dot_str = isam.dot(affected_cliques=affected)

        # Create pydot graph
        graph_list = pydot.graph_from_dot_data(dot_str)
        if graph_list and len(graph_list) > 0:
            graph = graph_list[0]

            # Save to temp file and load as image
            with tempfile.NamedTemporaryFile(suffix='.png', delete=False) as f:
                graph.write_png(f.name)
                img = plt.imread(f.name)
                os.unlink(f.name)

            tree_img.set_array(img)
            ax_tree.set_title(
                f"Step {frame}: Bayes Tree (Red = Updated Cliques)")
        else:
            ax_tree.set_title(
                f"Step {frame}: Bayes Tree (visualization unavailable)")
    except Exception as e:
        # If Bayes tree visualization fails, just show a message
        ax_tree.set_title(
            f"Step {frame}: Bayes Tree (error: {str(e)[:30]}...)")
        print(f"Warning: Could not visualize Bayes tree: {e}")

    return line_est, points_est, tree_img


ani = FuncAnimation(fig, update, frames=len(history),
                    interval=1500, blit=False, repeat=True)
plt.tight_layout()
plt.show()
