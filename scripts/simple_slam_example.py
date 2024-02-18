import gtsam
import numpy as np


def main():
    # Create an empty nonlinear factor graph
    graph = gtsam.NonlinearFactorGraph()

    # Create the initial estimate
    initial_estimate = gtsam.Values()

    # Add a prior on the first pose, setting it to the origin
    prior_pose = gtsam.Pose2(0.0, 0.0, 0.0)  # x, y, theta
    prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))
    graph.add(gtsam.PriorFactorPose2(1, prior_pose, prior_noise))

    # Add odometry factors
    odometry = gtsam.Pose2(2.0, 0.0, 0.0)  # x, y, theta
    odometry_noise = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([0.2, 0.2, 0.1]))
    graph.add(gtsam.BetweenFactorPose2(1, 2, odometry, odometry_noise))
    graph.add(gtsam.BetweenFactorPose2(2, 3, odometry, odometry_noise))

    # Initialize the poses based on the prior and odometry
    initial_estimate.insert(1, gtsam.Pose2(0.0, 0.0, 0.0))
    initial_estimate.insert(2, gtsam.Pose2(2.0, 0.0, 0.0))
    initial_estimate.insert(3, gtsam.Pose2(4.0, 0.0, 0.0))

    # Optimize using Levenberg-Marquardt optimization
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate)
    result = optimizer.optimize()

    # Output the optimized pose
    print("Final Result:")
    print(result)

    print("---------------------------------")
    print(graph)


if __name__ == "__main__":
    main()
