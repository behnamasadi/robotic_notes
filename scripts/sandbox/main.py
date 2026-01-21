# import numpy as np
# from scipy.stats import norm, multivariate_normal
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D


# x = np.linspace(-5, 5, 1000)
# y = np.linspace(-5, 5, 1000)
# X, Y = np.meshgrid(x, y)
# data = np.stack([X, Y], axis=-1).reshape(-1, 2)
# mu = np.array([0, 0])
# Sigma = np.array([[1, 0], [0, 1]])
# pdf_values = multivariate_normal.pdf(data, mean=mu, cov=Sigma).reshape(X.shape)

# fig = plt.figure(figsize=(10, 8))
# ax = fig.add_subplot(111, projection='3d')
# surf = ax.plot_surface(X, Y, pdf_values, cmap='viridis',
#                        alpha=0.8, linewidth=0, antialiased=True)
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('PDF')
# ax.set_title('Multivariate Normal PDF (3D)')
# fig.colorbar(surf, ax=ax, shrink=0.5, aspect=5)
# plt.show()


import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal, norm
from mpl_toolkits.mplot3d import Axes3D

# Grid
x = np.linspace(-5, 5, 400)
y = np.linspace(-5, 5, 400)
X, Y = np.meshgrid(x, y)

# Joint Gaussian parameters
mu = np.array([0.0, 0.0])
Sigma = np.array([[4.0, 1.0],
                  [1.0, 1.0]])

# Evaluate joint PDF
data = np.stack([X, Y], axis=-1).reshape(-1, 2)
Z = multivariate_normal.pdf(data, mean=mu, cov=Sigma).reshape(X.shape)

# Choose conditioning value
y0 = 1.5

# Conditional Gaussian parameters
mu_cond = y0
sigma2_cond = 4.0 - (1.0**2) / 1.0
sigma_cond = np.sqrt(sigma2_cond)

# Conditional PDF p(x | y=y0)
x_line = x
p_cond = norm.pdf(x_line, loc=mu_cond, scale=sigma_cond)

# --- Plot ---
fig = plt.figure(figsize=(16, 6))

# Left plot: 3D joint PDF
ax1 = fig.add_subplot(121, projection="3d")
surf = ax1.plot_surface(X, Y, Z, cmap="viridis", alpha=0.75)
ax1.set_xlabel("x")
ax1.set_ylabel("y")
ax1.set_zlabel("PDF")
ax1.set_title("Joint PDF $p(x, y)$")
fig.colorbar(surf, ax=ax1, shrink=0.6, aspect=10)

# Right plot: 2D conditional PDF
ax2 = fig.add_subplot(122)
ax2.plot(x_line, p_cond, color="red", linewidth=2,
         label=rf"$p(x \mid y={y0})$")
ax2.fill_between(x_line, p_cond, alpha=0.3, color="red")
ax2.set_xlabel("x")
ax2.set_ylabel("PDF")
ax2.set_title(f"Conditional PDF $p(x \mid y={y0})$")
ax2.grid(True, alpha=0.3)
ax2.legend()

plt.tight_layout()
plt.show()
