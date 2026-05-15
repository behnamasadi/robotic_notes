import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# Read first few poses
poses_file = Path(__file__).parent / "poses" / "00.txt"
poses_data = np.loadtxt(poses_file)

print("=" * 70)
print("UNDERSTANDING KITTI POSES")
print("=" * 70)

# Parse first 5 poses
print("\n📍 First 5 Poses Analysis:\n")

for i in range(5):
    T = np.eye(4)
    T[:3, :] = poses_data[i].reshape(3, 4)
    
    R = T[:3, :3]  # Rotation matrix
    t = T[:3, 3]   # Translation vector
    
    print(f"Pose {i}:")
    print(f"  Translation (x, y, z): [{t[0]:8.4f}, {t[1]:8.4f}, {t[2]:8.4f}] meters")
    
    # Check if rotation is close to identity
    is_identity = np.allclose(R, np.eye(3), atol=1e-6)
    if is_identity:
        print(f"  Rotation: IDENTITY (no rotation)")
    else:
        # Compute rotation angle
        trace = np.trace(R)
        angle = np.arccos(np.clip((trace - 1) / 2, -1, 1))
        print(f"  Rotation: {np.degrees(angle):.4f} degrees from identity")
    print()

print("=" * 70)
print("INTERPRETATION:")
print("=" * 70)
print("""
The poses in 00.txt represent:

T_w_i = Transformation from camera frame i to WORLD frame
      = Position and orientation of camera i in the world

Key Points:

1. ✅ Pose 0 (first pose) ≈ Identity Matrix
   - This means: Camera 0 DEFINES the world coordinate frame
   - The world origin is at Camera 0's position
   - World axes align with Camera 0's axes

2. 📸 Pose i (any other pose)
   - Shows where Camera i is relative to Camera 0
   - Translation t = [x, y, z] shows camera position in world (meters)
   - Rotation R shows camera orientation relative to world

3. 🔄 Relative Transform (for warping)
   - To warp from frame i to frame j, we need: T_i→j
   - Formula: T_i→j = T_w_j^(-1) · T_w_i
   - This converts: "both in world" → "relative to each other"

Example from your data:
- Pose 0: Camera at origin (world frame)
- Pose 1: Camera moved ~0.047m in X, ~0.028m in Y, ~0.859m in Z (forward)
- Pose 2: Camera moved ~0.094m in X, ~0.057m in Y, ~1.716m in Z (forward)

The camera is moving forward along Z-axis (typical driving scenario)!
""")

# Visualize camera trajectory
print("=" * 70)
print("Creating trajectory visualization...")
print("=" * 70)

# Extract all camera positions
num_poses = min(500, len(poses_data))  # First 500 frames
positions = []

for i in range(num_poses):
    T = np.eye(4)
    T[:3, :] = poses_data[i].reshape(3, 4)
    t = T[:3, 3]
    positions.append(t)

positions = np.array(positions)

# Create 3D plot
fig = plt.figure(figsize=(15, 10))

# Top view (X-Z plane)
ax1 = plt.subplot(2, 2, 1)
ax1.plot(positions[:, 0], positions[:, 2], 'b-', linewidth=1, alpha=0.7)
ax1.plot(positions[0, 0], positions[0, 2], 'go', markersize=15, label='Start (Pose 0 = World Origin)')
ax1.plot(positions[-1, 0], positions[-1, 2], 'ro', markersize=10, label=f'End (Pose {num_poses-1})')
ax1.set_xlabel('X (meters) - Right', fontsize=12)
ax1.set_ylabel('Z (meters) - Forward', fontsize=12)
ax1.set_title('Top View (Bird\'s Eye)', fontsize=14, fontweight='bold')
ax1.grid(True, alpha=0.3)
ax1.legend()
ax1.axis('equal')

# Side view (X-Y plane)
ax2 = plt.subplot(2, 2, 2)
ax2.plot(positions[:, 0], positions[:, 1], 'b-', linewidth=1, alpha=0.7)
ax2.plot(positions[0, 0], positions[0, 1], 'go', markersize=15, label='Start')
ax2.plot(positions[-1, 0], positions[-1, 1], 'ro', markersize=10, label='End')
ax2.set_xlabel('X (meters) - Right', fontsize=12)
ax2.set_ylabel('Y (meters) - Down', fontsize=12)
ax2.set_title('Side View', fontsize=14, fontweight='bold')
ax2.grid(True, alpha=0.3)
ax2.legend()
ax2.axis('equal')

# Front view (Y-Z plane)
ax3 = plt.subplot(2, 2, 3)
ax3.plot(positions[:, 2], positions[:, 1], 'b-', linewidth=1, alpha=0.7)
ax3.plot(positions[0, 2], positions[0, 1], 'go', markersize=15, label='Start')
ax3.plot(positions[-1, 2], positions[-1, 1], 'ro', markersize=10, label='End')
ax3.set_xlabel('Z (meters) - Forward', fontsize=12)
ax3.set_ylabel('Y (meters) - Down', fontsize=12)
ax3.set_title('Front View', fontsize=14, fontweight='bold')
ax3.grid(True, alpha=0.3)
ax3.legend()
ax3.axis('equal')

# 3D view
ax4 = plt.subplot(2, 2, 4, projection='3d')
ax4.plot(positions[:, 0], positions[:, 2], positions[:, 1], 'b-', linewidth=1, alpha=0.7)
ax4.scatter(positions[0, 0], positions[0, 2], positions[0, 1], 
           c='green', s=200, marker='o', label='Start (World Origin)')
ax4.scatter(positions[-1, 0], positions[-1, 2], positions[-1, 1], 
           c='red', s=100, marker='o', label='End')
ax4.set_xlabel('X (Right)', fontsize=10)
ax4.set_ylabel('Z (Forward)', fontsize=10)
ax4.set_zlabel('Y (Down)', fontsize=10)
ax4.set_title('3D Trajectory', fontsize=14, fontweight='bold')
ax4.legend()

plt.suptitle(f'KITTI Sequence 00: Camera Trajectory (first {num_poses} frames)', 
            fontsize=16, fontweight='bold')
plt.tight_layout()

output_path = Path(__file__).parent / "camera_trajectory.png"
plt.savefig(output_path, dpi=300, bbox_inches='tight')
print(f"✅ Saved trajectory plot to: {output_path}")

plt.show()

# Statistics
print("\n" + "=" * 70)
print("TRAJECTORY STATISTICS:")
print("=" * 70)
print(f"Total frames analyzed: {num_poses}")
print(f"Total distance traveled: {np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1)):.2f} meters")
print(f"Final position: X={positions[-1, 0]:.2f}m, Y={positions[-1, 1]:.2f}m, Z={positions[-1, 2]:.2f}m")
print(f"Bounding box:")
print(f"  X: [{positions[:, 0].min():.2f}, {positions[:, 0].max():.2f}] meters")
print(f"  Y: [{positions[:, 1].min():.2f}, {positions[:, 1].max():.2f}] meters")
print(f"  Z: [{positions[:, 2].min():.2f}, {positions[:, 2].max():.2f}] meters")

