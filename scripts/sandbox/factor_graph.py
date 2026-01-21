import gtsam
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import gtsam.utils.plot as gtsam_plot


def X(i):
    return gtsam.symbol('x', i)


def draw_uncertainty_ellipse(ax, pose, cov_2x2, n_std=2, color='#00FF00',
                             fill_alpha=0.2, edge_alpha=1.0, linewidth=2):
    """Draw uncertainty ellipse for a 2D pose.

    Args:
        ax: matplotlib axes
        pose: gtsam.Pose2 object
        cov_2x2: 2x2 covariance matrix for (x, y)
        n_std: number of standard deviations for ellipse
        color: ellipse color (default bright green)
        fill_alpha: transparency for fill
        edge_alpha: transparency for edge
        linewidth: line width for edge
    """
    x, y = pose.x(), pose.y()

    # Eigenvalue decomposition
    eigenvals, eigenvecs = np.linalg.eigh(cov_2x2)

    # Ensure positive eigenvalues (handle numerical issues)
    eigenvals = np.maximum(eigenvals, 1e-10)

    # Angle of rotation
    angle = np.degrees(np.arctan2(eigenvecs[1, 0], eigenvecs[0, 0]))

    # Width and height (scaled for 95% inliers in 2D: 2.447746830681)
    # See gtsam/utils/plot.py documentation
    scale_2d_95pct = 2.447746830681
    width = 2 * scale_2d_95pct * np.sqrt(eigenvals[0])
    height = 2 * scale_2d_95pct * np.sqrt(eigenvals[1])

    # Draw ellipse with bright green edge
    ellipse = Ellipse((x, y), width, height, angle=angle,
                      edgecolor=color, facecolor=color,
                      alpha=fill_alpha, linewidth=linewidth)
    ax.add_patch(ellipse)

    # Draw pose arrow (bright green, pointing in direction of theta)
    theta = pose.theta()
    arrow_length = 0.4
    dx = arrow_length * np.cos(theta)
    dy = arrow_length * np.sin(theta)
    ax.arrow(x, y, dx, dy, head_width=0.15, head_length=0.08,
             fc=color, ec=color, linewidth=2.5, alpha=1.0)


graph = gtsam.NonlinearFactorGraph()

# -----------------------------
# Prior on x_1
# -----------------------------
prior_mean = gtsam.Pose2(0.0, 0.0, 0.0)
prior_noise = gtsam.noiseModel.Diagonal.Sigmas(
    np.array([0.3, 0.3, 0.1])
)

graph.add(
    gtsam.PriorFactorPose2(
        X(1),
        prior_mean,
        prior_noise
    )
)

# -----------------------------
# Odometry factors
# -----------------------------
odometry = gtsam.Pose2(2.0, 0.0, 0.0)
odometry_noise = gtsam.noiseModel.Diagonal.Sigmas(
    np.array([0.2, 0.2, 0.1])
)

graph.add(
    gtsam.BetweenFactorPose2(
        X(1), X(2),
        odometry,
        odometry_noise
    )
)

graph.add(
    gtsam.BetweenFactorPose2(
        X(2), X(3),
        odometry,
        odometry_noise
    )
)

# -----------------------------
# GPS-like unary factors (x,y only)
# -----------------------------
gps_noise = gtsam.noiseModel.Diagonal.Sigmas(
    np.array([0.1, 0.1])   # 10 cm std on x,y
)

graph.add(
    gtsam.PoseTranslationPrior2D(
        X(1),
        gtsam.Pose2(0.0, 0.0, 0.0),
        gps_noise
    )
)

graph.add(
    gtsam.PoseTranslationPrior2D(
        X(2),
        gtsam.Pose2(2.0, 0.0, 0.0),
        gps_noise
    )
)

graph.add(
    gtsam.PoseTranslationPrior2D(
        X(3),
        gtsam.Pose2(4.0, 0.0, 0.0),
        gps_noise
    )
)

# -----------------------------
# Initial guesses
# -----------------------------
initial = gtsam.Values()
initial.insert(X(1), gtsam.Pose2(0.5, 0.0, 0.2))
initial.insert(X(2), gtsam.Pose2(2.3, 0.1, -0.2))
initial.insert(X(3), gtsam.Pose2(4.1, 0.1, 0.1))

# -----------------------------
# Compute marginals for initial values (before optimization)
# -----------------------------
marginals_before = gtsam.Marginals(graph, initial)

# -----------------------------
# Optimize with GPS
# -----------------------------
result_with_gps = gtsam.LevenbergMarquardtOptimizer(graph, initial).optimize()
result_with_gps.print()

# -----------------------------
# Compute marginals for optimized result with GPS
# -----------------------------
marginals_with_gps = gtsam.Marginals(graph, result_with_gps)

# -----------------------------
# Create graph without GPS for comparison
# -----------------------------
graph_no_gps = gtsam.NonlinearFactorGraph()
graph_no_gps.add(gtsam.PriorFactorPose2(X(1), prior_mean, prior_noise))
graph_no_gps.add(gtsam.BetweenFactorPose2(
    X(1), X(2), odometry, odometry_noise))
graph_no_gps.add(gtsam.BetweenFactorPose2(
    X(2), X(3), odometry, odometry_noise))

# Optimize without GPS
result_no_gps = gtsam.LevenbergMarquardtOptimizer(
    graph_no_gps, initial).optimize()
marginals_no_gps = gtsam.Marginals(graph_no_gps, result_no_gps)

print("*"*60)

# -----------------------------
# Visualize uncertainty ellipses: before, after (no GPS), after (with GPS)
# -----------------------------
fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(20, 6))
fig.patch.set_facecolor('black')


def plot_uncertainty(ax, title, values, marginals, poses_list):
    """Helper function to plot uncertainty ellipses."""
    ax.set_facecolor('black')
    ax.set_title(title, fontsize=14, color='white')
    ax.set_xlabel('X (m)', color='white')
    ax.set_ylabel('Y (m)', color='white')
    ax.tick_params(colors='white')
    ax.grid(True, alpha=0.2, color='gray')
    ax.set_aspect('equal')

    # Get all poses
    all_x = [values.atPose2(X(i)).x() for i in poses_list]
    all_y = [values.atPose2(X(i)).y() for i in poses_list]

    # Get covariance sizes to ensure ellipses fit
    max_ellipse_size = 0
    for i in poses_list:
        cov_full = marginals.marginalCovariance(X(i))
        cov_2x2 = cov_full[:2, :2]
        eigenvals = np.linalg.eigh(cov_2x2)[0]
        max_ellipse_size = max(max_ellipse_size, 2 *
                               2.447746830681 * np.sqrt(max(eigenvals)))

    ax.set_xlim(min(all_x) - max_ellipse_size - 0.5,
                max(all_x) + max_ellipse_size + 0.5)
    ax.set_ylim(min(all_y) - max_ellipse_size - 0.5,
                max(all_y) + max_ellipse_size + 0.5)

    for i in poses_list:
        pose = values.atPose2(X(i))
        cov_full = marginals.marginalCovariance(X(i))
        cov_2x2 = cov_full[:2, :2]
        draw_uncertainty_ellipse(
            ax, pose, cov_2x2, n_std=1, color='#00FF00',
            fill_alpha=0.2, edge_alpha=1.0)


# Plot 1: Before optimization
plot_uncertainty(ax1, 'Before Optimization', initial,
                 marginals_before, [1, 2, 3])

# Plot 2: After optimization (without GPS)
plot_uncertainty(ax2, 'After Optimization (No GPS)',
                 result_no_gps, marginals_no_gps, [1, 2, 3])

# Plot 3: After optimization (with GPS)
plot_uncertainty(ax3, 'After Optimization (With GPS)',
                 result_with_gps, marginals_with_gps, [1, 2, 3])

plt.tight_layout()
plt.show()

print(gtsam.symbol('x', 1))
