from pyquaternion import Quaternion
import numpy as np

# Define the quaternions
# Example rotation of B in A by 45 degrees around y-axis
Q_B_A = Quaternion(axis=[0, 1, 0], degrees=45)
# Example rotation of C in B by 30 degrees around x-axis
Q_C_B = Quaternion(axis=[1, 0, 0], degrees=30)

# Combined rotation
Q_C_A = Q_B_A * Q_C_B

print(f"Q_C_A (combined rotation): {Q_C_A}")

# Define the positions
P_B_A = np.array([1, 2, 3])  # Example position of B in A
P_C_B = np.array([2, 0, 1])  # Example position of C in B

# Combined translation
P_C_A = P_B_A + (Q_B_A.rotate(P_C_B))

print(f"P_C_A (combined position): {P_C_A}")
