# Exercises for the Lie series

One notebook per video of the series *from "why can't I add two rotations" to on-manifold
optimisation* (16 videos). Watch the video, then work through its notebook. Each has a few
exercises: a calculation by hand whose answer is a number the video says, a short proof, and a
little code on EuRoC MH_01 or on a simulation, the same data the video uses.

**Try each exercise before opening its solution.** Solutions are folded under *Solution*; every
number in them is printed by code in the notebook, not typed.

**Notation** is that of the paper the series follows, Solà, Deray & Atchuthan,
[*A micro Lie theory for state estimation in robotics*](../../../pdf/A_micro_Lie_theory_for_state_estimation_in_robotics.pdf)
(arXiv 1812.01537; [cheat sheet](../../../pdf/Lie_theory_cheat_sheet.pdf)): right-⊕ and right-⊖
unless a question says otherwise, and a pose tangent vector is $\boldsymbol\tau = [\boldsymbol\rho;\boldsymbol\theta]$,
translation first. Equation numbers in the notebooks are the paper's. The longer companion
notebook is [lie_group_lie_algebra.ipynb](../../lie_group_lie_algebra.ipynb).

**Code.** numpy, scipy and matplotlib. [`lie_tools.py`](lie_tools.py), next to the notebooks,
holds the few helpers they share (hat and vee, Exp and Log of SO(2), SO(3), SE(2), SE(3),
quaternions, the EuRoC loader). The exercises ask you to write most of them yourself first.

**Data.** EuRoC MH_01_easy (`imu0`, `state_groundtruth_estimate0`, and `cam0/sensor.yaml` for the
camera extrinsic), loaded the same way as the IMU-series exercises. Download MH_01_easy in the
ASL format from the dataset page, <https://ethz-asl.github.io/datasets/euroc-mav/> (the files are on
the ETH Research Collection, <https://doi.org/10.3929/ethz-b-000690084>), unzip it, and set
`EUROC_MAV0` to its `mav0` folder. Anything simulated is labelled.

| # | Video | Exercises | Notebook |
|---|---|---|---|
| 1 | Why You Can't Add Rotations, and What a Group Is | 6 | [01_rotations_not_vector_space_exercises.ipynb](01_rotations_not_vector_space_exercises.ipynb) |
| 2 | Rotations in 2D: the Whole Idea on One Circle | 5 | [02_so2_circle_exercises.ipynb](02_so2_circle_exercises.ipynb) |
| 3 | The Lie Algebra so(3): Skew Matrices, Hat and Vee, and Angular Velocity | 5 | [03_so3_tangent_space_exercises.ipynb](03_so3_tangent_space_exercises.ipynb) |
| 4 | The Exponential and Logarithm Maps on SO(3): Rodrigues' Formula | 5 | [04_exp_log_map_exercises.ipynb](04_exp_log_map_exercises.ipynb) |
| 5 | ⊕ and ⊖: Adding a Small Rotation, Subtracting Two Rotations | 5 | [05_lie_plus_minus_exercises.ipynb](05_lie_plus_minus_exercises.ipynb) |
| 6 | Unit Quaternions Are the Same Group: S³ and the Half Angle | 5 | [06_quaternion_s3_exercises.ipynb](06_quaternion_s3_exercises.ipynb) |
| 7 | SE(2): the Pose of a Planar Robot, and Why ρ Is Not t | 5 | [07_se2_pose_exercises.ipynb](07_se2_pose_exercises.ipynb) |
| 8 | SE(3): Poses in 3D, Twists and the Screw Motion | 6 | [08_so3_se3_exercises.ipynb](08_so3_se3_exercises.ipynb) |
| 9 | The Adjoint: the Same Small Motion, Seen from Another Frame | | [09_lie_adjoint_exercises.ipynb](09_lie_adjoint_exercises.ipynb) |
| 10 | Jacobians on Lie Groups: the Definition, and Right versus Left | | [10_manifold_jacobians_exercises.ipynb](10_manifold_jacobians_exercises.ipynb) |
| 11 | Inverse, Composition and the Chain Rule: Building a Jacobian from Blocks | | [11_lie_jacobian_blocks_exercises.ipynb](11_lie_jacobian_blocks_exercises.ipynb) |
| 12 | The Right Jacobian of SO(3): the Derivative of Exp, Log, ⊕ and ⊖ | | [12_right_jacobian_exercises.ipynb](12_right_jacobian_exercises.ipynb) |
| 13 | Uncertainty on a Manifold: a Covariance in the Tangent Space | | [13_manifold_covariance_exercises.ipynb](13_manifold_covariance_exercises.ipynb) |
| 14 | A Kalman Filter on a Lie Group: Localising a Robot on SE(2) | | [14_se2_localization_exercises.ipynb](14_se2_localization_exercises.ipynb) |
| 15 | Gauss–Newton on a Lie Group: Smoothing and Mapping | | [15_se2_sam_exercises.ipynb](15_se2_sam_exercises.ipynb) |
| 16 | On-Manifold Optimisation in Code: manif and Ceres | | [16_manif_exercises.ipynb](16_manif_exercises.ipynb) |
