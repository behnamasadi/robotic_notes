import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import numpy as np

fig, ax = plt.subplots(1, 1, figsize=(14, 10))
ax.set_xlim(0, 10)
ax.set_ylim(0, 12)
ax.axis('off')

# Title
ax.text(5, 11.5, 'Photometric Loss Pipeline', 
        ha='center', va='top', fontsize=18, fontweight='bold')

# Data inputs
def draw_box(ax, x, y, w, h, text, color='lightblue'):
    box = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.1", 
                          edgecolor='black', facecolor=color, linewidth=2)
    ax.add_patch(box)
    ax.text(x + w/2, y + h/2, text, ha='center', va='center', 
            fontsize=10, fontweight='bold', wrap=True)

def draw_arrow(ax, x1, y1, x2, y2, label=''):
    arrow = FancyArrowPatch((x1, y1), (x2, y2), 
                           arrowstyle='->', mutation_scale=20, 
                           linewidth=2, color='black')
    ax.add_patch(arrow)
    if label:
        mid_x, mid_y = (x1 + x2) / 2, (y1 + y2) / 2
        ax.text(mid_x + 0.3, mid_y, label, fontsize=8, style='italic')

# Input data
draw_box(ax, 0.5, 10, 1.5, 0.8, 'I_t\n(Target\nImage)', 'lightgreen')
draw_box(ax, 2.5, 10, 1.5, 0.8, 'I_s\n(Source\nImage)', 'lightgreen')
draw_box(ax, 4.5, 10, 1.5, 0.8, 'D_t\n(Depth)', 'lightyellow')
draw_box(ax, 6.5, 10, 1.5, 0.8, 'K\n(Intrinsics)', 'lightcoral')
draw_box(ax, 8.5, 10, 1.5, 0.8, 'Poses\nT_w_i, T_w_j', 'lightcoral')

# Step 1: Compute relative pose
draw_box(ax, 7, 8.5, 2.5, 0.8, 'T_t→s = T_w_j^(-1) · T_w_i', 'lightskyblue')
draw_arrow(ax, 8.5, 10, 8.25, 9.3)
draw_arrow(ax, 9.5, 10, 8.25, 9.3)

# Step 2: Back-projection
draw_box(ax, 1, 7, 3, 0.8, 'Back-project:\nX_cam = D·K^(-1)·[u,v,1]^T', 'wheat')
draw_arrow(ax, 4.5, 10, 2.5, 7.8, 'D_t')
draw_arrow(ax, 7, 10, 4, 7.5, 'K')

# Step 3: Transform
draw_box(ax, 5.5, 7, 3, 0.8, 'Transform:\nX_s = T_t→s · X_t', 'wheat')
draw_arrow(ax, 4, 7.4, 5.5, 7.4)
draw_arrow(ax, 8.25, 8.5, 7, 7.6)

# Step 4: Project
draw_box(ax, 1, 5.5, 3, 0.8, 'Project:\n[u\',v\'] = π(K·X_s)', 'wheat')
draw_arrow(ax, 7, 7.4, 4, 6, 'X_s')
draw_arrow(ax, 7, 9.5, 4, 6.3, 'K')

# Step 5: Warp/Sample
draw_box(ax, 5.5, 5.5, 3, 0.8, 'Sample:\nI_warp = I_s(u\',v\')', 'wheat')
draw_arrow(ax, 4, 5.9, 5.5, 5.9)
draw_arrow(ax, 3.25, 10, 7, 6.3, 'I_s')

# Result: Warped image
draw_box(ax, 3, 4, 3, 0.8, 'I_warped\n(Warped Image)', 'lightgreen')
draw_arrow(ax, 7, 5.9, 6, 4.5)

# SSIM branch
draw_box(ax, 0.5, 2.5, 2, 0.6, 'SSIM Loss\nL_SSIM', 'lavender')
draw_arrow(ax, 1.25, 10, 1, 3.1, 'I_t')
draw_arrow(ax, 4.5, 4, 2.5, 2.8)

# L1 branch
draw_box(ax, 5.5, 2.5, 2, 0.6, 'L1 Loss\n|I_t - I_warp|', 'lavender')
draw_arrow(ax, 1.25, 10, 6.5, 3.1, 'I_t')
draw_arrow(ax, 4.5, 4, 5.5, 2.8)

# Combine
draw_box(ax, 2.5, 1, 3.5, 0.6, 'Combine: 0.85·L_SSIM + 0.15·L_L1', 'salmon')
draw_arrow(ax, 1.5, 2.5, 4, 1.6)
draw_arrow(ax, 6.5, 2.5, 6, 1.6)

# Final loss
draw_box(ax, 3, 0.1, 2.5, 0.6, 'Photometric\nLoss', 'lightcoral')
draw_arrow(ax, 4.25, 1, 4.25, 0.7)

# Add legend with equations
equation_text = """
Key Equations:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Back-projection:  X = D(u,v)·(u-cx)/fx
                  Y = D(u,v)·(v-cy)/fy  
                  Z = D(u,v)

Transform:        X_s = R·X_t + t

Projection:       u' = fx·(X_s/Z_s) + cx
                  v' = fy·(Y_s/Z_s) + cy

Photometric:      L = α·SSIM + (1-α)·L1
"""

ax.text(10.5, 6, equation_text, fontsize=8, family='monospace',
        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3))

plt.tight_layout()
plt.savefig('/home/behnam/anaconda3/envs/PyTorchTutorial/src/SLAM/pipeline_diagram.png', 
            dpi=300, bbox_inches='tight')
print("✅ Pipeline diagram saved!")
plt.close()

# Create second diagram for coordinate systems
fig2, axes = plt.subplots(1, 3, figsize=(15, 5))

# World coordinate system
ax1 = axes[0]
ax1.set_xlim(-2, 2)
ax1.set_ylim(-2, 2)
ax1.set_aspect('equal')
ax1.arrow(0, 0, 1.5, 0, head_width=0.1, head_length=0.1, fc='red', ec='red')
ax1.arrow(0, 0, 0, 1.5, head_width=0.1, head_length=0.1, fc='green', ec='green')
ax1.text(1.7, 0, 'X_w', fontsize=12, color='red', fontweight='bold')
ax1.text(0, 1.7, 'Y_w', fontsize=12, color='green', fontweight='bold')
ax1.text(0, -0.3, 'Z_w (out)', fontsize=10, ha='center')
ax1.set_title('World Coordinate System', fontsize=14, fontweight='bold')
ax1.grid(True, alpha=0.3)
ax1.axhline(y=0, color='k', linewidth=0.5)
ax1.axvline(x=0, color='k', linewidth=0.5)

# Camera coordinate system
ax2 = axes[1]
ax2.set_xlim(-2, 2)
ax2.set_ylim(-2, 2)
ax2.set_aspect('equal')
ax2.arrow(0, 0, 1.5, 0, head_width=0.1, head_length=0.1, fc='red', ec='red')
ax2.arrow(0, 0, 0, -1.5, head_width=0.1, head_length=0.1, fc='green', ec='green')
ax2.text(1.7, 0, 'X_c (right)', fontsize=12, color='red', fontweight='bold')
ax2.text(0, -1.7, 'Y_c (down)', fontsize=12, color='green', fontweight='bold')
ax2.text(0, 0.3, 'Z_c (forward)', fontsize=10, ha='center')
ax2.plot(0, 0, 'ko', markersize=10)
ax2.text(0.2, 0.2, 'Camera\nCenter', fontsize=9)
ax2.set_title('Camera Coordinate System', fontsize=14, fontweight='bold')
ax2.grid(True, alpha=0.3)
ax2.axhline(y=0, color='k', linewidth=0.5)
ax2.axvline(x=0, color='k', linewidth=0.5)

# Image coordinate system
ax3 = axes[2]
ax3.set_xlim(0, 10)
ax3.set_ylim(10, 0)  # Inverted Y
ax3.arrow(0, 0, 8, 0, head_width=0.3, head_length=0.3, fc='red', ec='red')
ax3.arrow(0, 0, 0, 8, head_width=0.3, head_length=0.3, fc='green', ec='green')
ax3.text(8.5, 0, 'u (cols)', fontsize=12, color='red', fontweight='bold')
ax3.text(0, 8.5, 'v (rows)', fontsize=12, color='green', fontweight='bold')
ax3.plot(5, 4, 'bo', markersize=8)
ax3.text(5.5, 4, '(cx, cy)\nPrincipal\nPoint', fontsize=9)
ax3.text(0.5, 0.5, '(0,0)\nOrigin', fontsize=9)
ax3.set_title('Image Coordinate System (Pixels)', fontsize=14, fontweight='bold')
ax3.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('/home/behnam/anaconda3/envs/PyTorchTutorial/src/SLAM/coordinate_systems.png', 
            dpi=300, bbox_inches='tight')
print("✅ Coordinate systems diagram saved!")

