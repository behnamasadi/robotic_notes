import numpy as np
import matplotlib.pyplot as plt


def gaussian(x, mu, sig):
    return (1.0 / (np.sqrt(2 * np.pi) * sig)) * \
        np.exp(-0.5 * ((x - mu) / sig)**2)


x = np.linspace(-6, 6, 1000)
mu = 1.0
sig = 1.2

p_x = gaussian(x, mu, sig)

mu_vals = np.linspace(-6, 6, 1000)
x0 = 2.0

likelihood = gaussian(x0, mu_vals, sig)

# Create side-by-side plots
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))

# Left plot: Probability Density Function
ax1.plot(x, p_x, linewidth=2)
ax1.set_xlabel("x")
ax1.set_ylabel("p(x | μ, σ)")
ax1.set_title("Probability Density Function")
ax1.grid(True)

# Right plot: Likelihood Function
ax2.plot(mu_vals, likelihood, linewidth=2)
ax2.set_xlabel("μ")
ax2.set_ylabel("L(μ | x₀)")
ax2.set_title("Likelihood Function")
ax2.grid(True)

plt.tight_layout()
plt.show()
