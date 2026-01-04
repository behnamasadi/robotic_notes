"""
Example: Loading and optimizing a g2o file using GTSAM

GTSAM provides readG2o() function to load g2o format files directly.
This is simpler than manual parsing.
"""

import gtsam
import numpy as np
import argparse
import os


def load_and_optimize_g2o(g2o_file, is_3d=False):
    """
    Load a g2o file and optimize it using GTSAM.

    Parameters:
    -----------
    g2o_file : str
        Path to the .g2o file
    is_3d : bool
        True for 3D pose graphs (Pose3), False for 2D (Pose2)

    Returns:
    --------
    graph : NonlinearFactorGraph
        The factor graph from the g2o file
    initial : Values
        Initial estimate from the g2o file
    result : Values
        Optimized result
    """

    # Load g2o file - returns (graph, initial_estimate)
    graph, initial = gtsam.readG2o(g2o_file, is3D=is_3d)

    print(f"Loaded g2o file: {g2o_file}")
    print(f"Number of factors: {graph.size()}")
    print(f"Number of variables: {initial.size()}")

    # Add a prior on the first pose to anchor the solution
    # This fixes the gauge freedom (translation and rotation ambiguity)
    if not is_3d:
        # 2D case
        prior_noise = gtsam.noiseModel.Diagonal.Variances(
            np.array([1e-6, 1e-6, 1e-8], dtype=np.float64)
        )
        graph.add(gtsam.PriorFactorPose2(0, gtsam.Pose2(), prior_noise))
    else:
        # 3D case
        prior_noise = gtsam.noiseModel.Diagonal.Variances(
            np.array([1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-8], dtype=np.float64)
        )
        graph.add(gtsam.PriorFactorPose3(0, gtsam.Pose3(), prior_noise))

    # Optimize using Gauss-Newton
    params = gtsam.GaussNewtonParams()
    params.setMaxIterations(100)
    params.setVerbosity("TERMINATION")  # Show optimization progress

    optimizer = gtsam.GaussNewtonOptimizer(graph, initial, params)
    result = optimizer.optimize()

    print(f"\nOptimization completed!")
    print(f"Final error: {graph.error(result):.6f}")

    return graph, initial, result


def load_g2o_with_isam2(g2o_file, is_3d=False):
    """
    Load a g2o file and use iSAM2 for incremental optimization.
    This is useful for large graphs or online SLAM.
    """

    # Load g2o file
    graph, initial = gtsam.readG2o(g2o_file, is3D=is_3d)

    # Set up iSAM2
    params = gtsam.ISAM2Params()
    params.setRelinearizeThreshold(0.01)
    params.relinearizeSkip = 1
    isam = gtsam.ISAM2(params)

    # Add a prior on the first pose
    prior_graph = gtsam.NonlinearFactorGraph()
    if not is_3d:
        prior_noise = gtsam.noiseModel.Diagonal.Variances(
            np.array([1e-6, 1e-6, 1e-8], dtype=np.float64)
        )
        prior_graph.add(gtsam.PriorFactorPose2(0, gtsam.Pose2(), prior_noise))
    else:
        prior_noise = gtsam.noiseModel.Diagonal.Variances(
            np.array([1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-8], dtype=np.float64)
        )
        prior_graph.add(gtsam.PriorFactorPose3(0, gtsam.Pose3(), prior_noise))

    # Update iSAM2 with the graph
    isam.update(graph, initial)
    result = isam.calculateEstimate()

    print(f"iSAM2 optimization completed!")
    print(f"Final error: {graph.error(result):.6f}")

    return graph, initial, result


if __name__ == "__main__":
    # Set up argument parser
    parser = argparse.ArgumentParser(
        description="Load and optimize a g2o file using GTSAM",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Load 2D g2o file with default path
  python load_g2o_gtsam_example.py
  
  # Load custom 2D g2o file
  python load_g2o_gtsam_example.py --input path/to/file.g2o
  
  # Load 3D g2o file
  python load_g2o_gtsam_example.py --input path/to/file.g2o --3d
  
  # Use iSAM2 instead of Gauss-Newton
  python load_g2o_gtsam_example.py --isam2
        """
    )

    parser.add_argument(
        "--input", "-i",
        type=str,
        default="/home/behnam/workspace/robotic_notes/data/slam/input_INTEL_g2o.g2o",
        help="Path to the g2o file (default: input_INTEL_g2o.g2o)"
    )

    parser.add_argument(
        "--3d",
        action="store_true",
        dest="is_3d",
        help="Load as 3D pose graph (Pose3) instead of 2D (Pose2)"
    )

    parser.add_argument(
        "--isam2",
        action="store_true",
        help="Use iSAM2 for incremental optimization instead of Gauss-Newton"
    )

    args = parser.parse_args()

    # Check if file exists
    if not os.path.exists(args.input):
        print(f"Error: File not found: {args.input}")
        exit(1)

    # Load and optimize
    if args.isam2:
        print("=== Using iSAM2 (Incremental Optimization) ===")
        graph, initial, result = load_g2o_with_isam2(
            args.input, is_3d=args.is_3d)
    else:
        print("=== Using Gauss-Newton Optimizer (Batch Optimization) ===")
        graph, initial, result = load_and_optimize_g2o(
            args.input, is_3d=args.is_3d)
