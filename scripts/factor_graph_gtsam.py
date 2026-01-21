
import numpy as np
import matplotlib.pyplot as plt

import gtsam
from gtsam import symbol

# -----------------------------
# Helpers
# -----------------------------


def sigmas_from_information(diag_info):
    diag_info = np.asarray(diag_info, dtype=float)
    return 1.0 / np.sqrt(diag_info)


def values_sorted_keys(values: gtsam.Values):
    return sorted([k for k in values.keys()])


def values_to_xytheta(values: gtsam.Values, keys):
    xs, ys, th = [], [], []
    for k in keys:
        p = values.atPose2(k)
        xs.append(p.x())
        ys.append(p.y())
        th.append(p.theta())
    return np.array(xs), np.array(ys), np.array(th)


def plot_step(values: gtsam.Values, odom_edges, loop_edges, title):
    keys = values_sorted_keys(values)
    xs, ys, th = values_to_xytheta(values, keys)

    fig, ax = plt.subplots(figsize=(7, 4))

    # trajectory line through sorted keys
    ax.plot(xs, ys, marker="o", linestyle="-")

    # orientation arrows
    ax.quiver(xs, ys, np.cos(th), np.sin(th),
              angles="xy", scale_units="xy", scale=3)

    # draw factor edges
    def draw_edges(edges, style="-", alpha=0.6):
        for (a, b) in edges:
            pa = values.atPose2(a)
            pb = values.atPose2(b)
            ax.plot([pa.x(), pb.x()], [pa.y(), pb.y()], style, alpha=alpha)

    draw_edges(odom_edges, style="-", alpha=0.25)
    draw_edges(loop_edges, style="--", alpha=0.95)

    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)
    ax.set_title(title)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    plt.show()


# -----------------------------
# Problem data (your numbers)
# -----------------------------
N = 10
def X(i): return symbol("x", i)


# Information matrices (diagonal)
Omega_odom = [400, 400, 400]      # sigmas -> [0.05, 0.05, 0.05]
Omega_loop = [2500, 2500, 2500]   # sigmas -> [0.02, 0.02, 0.02]
Omega_0 = [1e6, 1e6, 1e6]      # sigmas -> [0.001, 0.001, 0.001]

sig_odom = sigmas_from_information(Omega_odom)
sig_loop = sigmas_from_information(Omega_loop)
sig_0 = sigmas_from_information(Omega_0)

noise_odom = gtsam.noiseModel.Diagonal.Sigmas(sig_odom)
noise_loop = gtsam.noiseModel.Diagonal.Sigmas(sig_loop)
noise_0 = gtsam.noiseModel.Diagonal.Sigmas(sig_0)

# Measurements as Pose2 (dx, dy, dtheta)
z0 = gtsam.Pose2(0.0, 0.0, 0.0)
z_odom = gtsam.Pose2(1.0, 0.0, 0.01)
z_loop_6_9 = gtsam.Pose2(3.0, 0.10, 0.0)

# -----------------------------
# iSAM2 setup
# -----------------------------
isam_params = gtsam.ISAM2Params()
# A couple of sensible defaults for this toy problem:
isam_params.setRelinearizeThreshold(0.01)
isam_params.relinearizeSkip = 1
isam = gtsam.ISAM2(isam_params)

odom_edges = []
loop_edges = []

# Keep an "initial guesses" container you can extend incrementally
initial = gtsam.Values()


def isam_update_and_plot(new_factors: gtsam.NonlinearFactorGraph,
                         new_values: gtsam.Values,
                         title: str):
    # Update iSAM2 with the newly added factors/values
    isam.update(new_factors, new_values)
    # You can call update() again to force additional relinearization if desired:
    # isam.update()

    # Current best estimate
    estimate = isam.calculateEstimate()
    plot_step(estimate, odom_edges, loop_edges, title)

    return estimate


# -----------------------------
# Step 0: prior + X0
# -----------------------------
new_factors = gtsam.NonlinearFactorGraph()
new_values = gtsam.Values()

new_factors.add(gtsam.PriorFactorPose2(X(0), z0, noise_0))
new_values.insert(X(0), z0)
initial.insert(X(0), z0)

est = isam_update_and_plot(new_factors, new_values,
                           "Step 0: add prior on X0 (iSAM2 estimate)")

# -----------------------------
# Steps 1..9: add odometry factors one by one
# -----------------------------
for i in range(N - 1):
    new_factors = gtsam.NonlinearFactorGraph()
    new_values = gtsam.Values()

    # add odom factor
    new_factors.add(gtsam.BetweenFactorPose2(
        X(i), X(i + 1), z_odom, noise_odom))
    odom_edges.append((X(i), X(i + 1)))

    # initial guess for new node by chaining from current estimate (more stable than chaining initial)
    est_i = isam.calculateEstimate().atPose2(X(i))
    guess_next = est_i.compose(z_odom)

    new_values.insert(X(i + 1), guess_next)
    initial.insert(X(i + 1), guess_next)

    est = isam_update_and_plot(
        new_factors,
        new_values,
        f"Step {i+1}: add odom X{i}→X{i+1} (iSAM2 estimate)"
    )

# Before loop closure: plot current estimate
plot_step(isam.calculateEstimate(), odom_edges, loop_edges,
          "Before loop closure: iSAM2 estimate (odom only)")

# -----------------------------
# Add loop closure 6 -> 9
# -----------------------------
new_factors = gtsam.NonlinearFactorGraph()
new_values = gtsam.Values()  # no new nodes, so empty is fine

new_factors.add(gtsam.BetweenFactorPose2(X(6), X(9), z_loop_6_9, noise_loop))
loop_edges.append((X(6), X(9)))

est = isam_update_and_plot(new_factors, new_values,
                           "After loop closure: iSAM2 estimate")

# -----------------------------
# Print final poses
# -----------------------------
final_est = isam.calculateEstimate()
print("Final iSAM2 poses:")
for i in range(N):
    p = final_est.atPose2(X(i))
    print(f"X{i}: x={p.x(): .4f}, y={p.y(): .4f}, theta={p.theta(): .4f} rad")


# import gtsam
# import numpy as np
# import math
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
# import pydot
# import tempfile
# import os
# import argparse

# # ----------------------------- Load g2o File -----------------------------


# def load_g2o_file(g2o_file):
#     """
#     Load a g2o file and return the graph and initial estimates.
#     """
#     print(f"Loading g2o file: {g2o_file}")
#     graph, initial = gtsam.readG2o(g2o_file, is3D=False)
#     print(f"Loaded {graph.size()} factors and {initial.size()} variables")
#     return graph, initial


# # ----------------------------- Setup -----------------------------
# # Parse command line arguments
# parser = argparse.ArgumentParser(
#     description="Load and visualize a g2o file using GTSAM iSAM2",
#     formatter_class=argparse.RawDescriptionHelpFormatter
# )
# parser.add_argument(
#     "--input", "-i",
#     type=str,
#     default="/home/behnam/workspace/robotic_notes/data/slam/input_INTEL_g2o.g2o",
#     help="Path to the g2o file"
# )
# parser.add_argument(
#     "--incremental",
#     action="store_true",
#     help="Process factors incrementally (slower but shows incremental updates)"
# )
# args = parser.parse_args()

# # Check if file exists
# if not os.path.exists(args.input):
#     print(f"Error: File not found: {args.input}")
#     exit(1)

# # Load g2o file
# full_graph, initial_estimate = load_g2o_file(args.input)

# # Set up iSAM2
# params = gtsam.ISAM2Params()
# params.setRelinearizeThreshold(0.1)
# params.relinearizeSkip = 1
# isam = gtsam.ISAM2(params)

# # Add a prior on the first pose to anchor the solution
# prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01]))
# prior_graph = gtsam.NonlinearFactorGraph()
# if initial_estimate.exists(0):
#     first_pose = initial_estimate.atPose2(0)
#     prior_graph.add(gtsam.PriorFactorPose2(0, first_pose, prior_noise))
# else:
#     # If pose 0 doesn't exist, use identity
#     prior_graph.add(gtsam.PriorFactorPose2(0, gtsam.Pose2(), prior_noise))

# # Storage for animation
# history = []  # List of (result, affected_cliques) tuples

# # ----------------------------- Incremental Solve -----------------------------
# if args.incremental:
#     # Process factors incrementally (for visualization)
#     print("\nProcessing factors incrementally...")

#     # Start with prior
#     isam.update(prior_graph, gtsam.Values())
#     result = isam.calculateEstimate()
#     affected = set()
#     result_copy = gtsam.Values(result)
#     history.append((result_copy, affected))

#     # Process factors in batches for visualization
#     batch_size = max(1, full_graph.size() // 20)  # 20 steps for animation

#     for i in range(0, full_graph.size(), batch_size):
#         batch_graph = gtsam.NonlinearFactorGraph()
#         batch_initial = gtsam.Values()

#         # Add factors in batch
#         for j in range(i, min(i + batch_size, full_graph.size())):
#             factor = full_graph.at(j)
#             batch_graph.add(factor)

#             # Extract keys from factor and add initial estimates if needed
#             keys = factor.keys()
#             for key in keys:
#                 if not result.exists(key) and initial_estimate.exists(key):
#                     batch_initial.insert(key, initial_estimate.atPose2(key))

#         if batch_graph.size() > 0:
#             isam.update(batch_graph, batch_initial)
#             result = isam.calculateEstimate()
#             affected = set()
#             result_copy = gtsam.Values(result)
#             history.append((result_copy, affected))
#             print(
#                 f"Processed {min(i + batch_size, full_graph.size())}/{full_graph.size()} factors")
# else:
#     # Process all at once (faster)
#     print("\nProcessing all factors at once...")
#     # Add prior to the graph
#     for i in range(prior_graph.size()):
#         full_graph.add(prior_graph.at(i))

#     isam.update(full_graph, initial_estimate)
#     result = isam.calculateEstimate()
#     affected = set()
#     result_copy = gtsam.Values(result)
#     history.append((result_copy, affected))
#     print("Optimization completed!")

# print(f"Final error: {full_graph.error(result):.6f}")

# # ----------------------------- Visualization -----------------------------
# # Get final result to determine plot limits
# final_result = history[-1][0] if history else result

# # Extract keys - try different methods
# try:
#     keys_list = list(final_result.keys())
# except:
#     # Alternative: iterate until we can't find a key
#     keys_list = []
#     i = 0
#     while final_result.exists(i):
#         keys_list.append(i)
#         i += 1

# if len(keys_list) == 0:
#     print("Error: No poses found in the graph")
#     print(f"Values size: {final_result.size()}")
#     exit(1)

# print(f"Found {len(keys_list)} poses in the graph")

# # Calculate trajectory bounds
# x_coords = []
# y_coords = []
# for key in keys_list:
#     try:
#         pose = final_result.atPose2(key)
#         x_coords.append(pose.x())
#         y_coords.append(pose.y())
#     except:
#         continue

# if len(x_coords) == 0:
#     print("Error: Could not extract any Pose2 values")
#     exit(1)

# x_min, x_max = min(x_coords), max(x_coords)
# y_min, y_max = min(y_coords), max(y_coords)
# x_margin = (x_max - x_min) * 0.1 if (x_max - x_min) > 0 else 1.0
# y_margin = (y_max - y_min) * 0.1 if (y_max - y_min) > 0 else 1.0

# fig, (ax_traj, ax_tree) = plt.subplots(1, 2, figsize=(14, 7))
# ax_traj.set_xlim(x_min - x_margin, x_max + x_margin)
# ax_traj.set_ylim(y_min - y_margin, y_max + y_margin)
# ax_traj.set_aspect('equal')
# ax_traj.grid(True)
# ax_traj.set_title("Trajectory Estimate")
# ax_traj.set_xlabel("X (m)")
# ax_traj.set_ylabel("Y (m)")

# line_est, = ax_traj.plot([], [], 'b-', linewidth=2, label="Current estimate")
# points_est, = ax_traj.plot([], [], 'bo')
# ax_traj.legend()

# # Bayes tree image placeholder
# tree_img = ax_tree.imshow(np.zeros((100, 100, 3)), aspect='equal')
# ax_tree.axis('off')
# ax_tree.set_title("Bayes Tree (Red = Updated Cliques)")


# def update(frame):
#     result, affected = history[frame]

#     # Trajectory - extract keys
#     try:
#         keys_list = list(result.keys())
#     except:
#         keys_list = []
#         i = 0
#         while result.exists(i):
#             keys_list.append(i)
#             i += 1

#     # Extract x, y coordinates
#     x = []
#     y = []
#     for key in keys_list:
#         try:
#             pose = result.atPose2(key)
#             x.append(pose.x())
#             y.append(pose.y())
#         except:
#             continue

#     if len(x) > 0:
#         line_est.set_data(x, y)
#         points_est.set_data(x, y)

#     # Bayes tree - try to visualize, but handle failures gracefully
#     try:
#         # Try using isam directly (it inherits from BayesTree)
#         dot_str = isam.dot(affected_cliques=affected)

#         # Create pydot graph
#         graph_list = pydot.graph_from_dot_data(dot_str)
#         if graph_list and len(graph_list) > 0:
#             graph = graph_list[0]

#             # Save to temp file and load as image
#             with tempfile.NamedTemporaryFile(suffix='.png', delete=False) as f:
#                 graph.write_png(f.name)
#                 img = plt.imread(f.name)
#                 os.unlink(f.name)

#             tree_img.set_array(img)
#             ax_tree.set_title(
#                 f"Step {frame}: Bayes Tree (Red = Updated Cliques)")
#         else:
#             ax_tree.set_title(
#                 f"Step {frame}: Bayes Tree (visualization unavailable)")
#     except Exception as e:
#         # If Bayes tree visualization fails, just show a message
#         ax_tree.set_title(
#             f"Step {frame}: Bayes Tree (error: {str(e)[:30]}...)")
#         print(f"Warning: Could not visualize Bayes tree: {e}")

#     return line_est, points_est, tree_img


# ani = FuncAnimation(fig, update, frames=len(history),
#                     interval=1500, blit=False, repeat=True)
# plt.tight_layout()
# plt.show()
