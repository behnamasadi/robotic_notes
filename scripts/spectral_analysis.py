from scipy.signal import welch
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats

# Generate a signal
t = np.linspace(0, 1, 1000, False)  # 1 second
# sig = np.sin(2*np.pi*7*t) + np.sin(2*np.pi*13*t)
sig = np.random.randn(1000)

# Plot signal
plt.figure()
plt.plot(t, sig)
plt.title('Signal')
plt.xlabel('Time [s]')
plt.ylabel('Amplitude')
plt.show()


#  Normal distribution (bell curve)


plt.hist(sig, bins=50, density=True, alpha=0.7, edgecolor='black')
x_norm = np.linspace(sig.min(), sig.max(), 100)
pdf_norm = stats.norm.pdf(x_norm, loc=0, scale=1)
plt.plot(x_norm, pdf_norm, 'r--', linewidth=2,
         label='Theoretical PDF (normal)')
plt.title('PDF of np.random.randn(1000)\n(Normal Distribution - Bell Curve)')
plt.xlabel('Value')
plt.ylabel('Density')
plt.legend()
plt.show()


# Compute PSD
fs = 1000  # Sampling frequency
f, Pxx = welch(sig, fs=fs, scaling='density')
# Calculate variance (sigma^2) of the signal
sigma_squared = np.var(sig)
mean_psd = np.mean(Pxx)

# For Welch's method with 'density' scaling, the PSD of white noise
# should be approximately σ²/fs (variance divided by sampling frequency)
expected_psd = sigma_squared / fs

print(f"Signal variance (σ²): {sigma_squared:.4f}")
print(f"Sampling frequency (fs): {fs} Hz")
print(f"Mean PSD value: {mean_psd:.6f}")
print(f"Expected PSD for white noise (σ²/fs): {expected_psd:.6f}")
print(f"Ratio (mean_psd / expected): {mean_psd / expected_psd:.4f}")

# Plot PSD
plt.figure()
plt.semilogy(f, Pxx, label='PSD (Welch estimate)', linewidth=1.5)
plt.axhline(y=expected_psd, color='r', linestyle='--', linewidth=2,
            label=f'Theoretical PSD = σ²/fs = {expected_psd:.6f}')
plt.title(
    f'PSD of White Gaussian Noise\n(Should be flat at σ²/fs = {expected_psd:.6f})')
plt.xlabel('Frequency [Hz]')
plt.ylabel('Power/Frequency [V**2/Hz]')
plt.legend()
plt.grid(True, alpha=0.3)
plt.show()

# Compute integral (cumulative sum) to illustrate random walk
dt = t[1] - t[0]  # Time step
sig_integrated = np.cumsum(sig) * dt  # Integral = cumulative sum * dt

# Plot integrated signal (random walk)
plt.figure()
plt.plot(t, sig_integrated, linewidth=1.5)
plt.title('Integrated Signal (Random Walk)')
plt.xlabel('Time [s]')
plt.ylabel('Cumulative Value')
plt.grid(True, alpha=0.3)
plt.show()

# Compute PSD of integrated signal (random walk has 1/f² PSD)
f_integrated, Pxx_integrated = welch(sig_integrated, fs=fs, scaling='density')

# Plot PSD of integrated signal
plt.figure()
plt.loglog(f_integrated, Pxx_integrated,
           label='PSD of Random Walk', linewidth=1.5)
# Reference line for 1/f² behavior
f_ref = f_integrated[f_integrated > 0]
psd_ref = Pxx_integrated[0] * (f_ref[0] / f_ref)**2
plt.loglog(f_ref, psd_ref, 'r--', linewidth=2, label='1/f² reference')
plt.title('PSD of Integrated Signal (Random Walk)\n(Should follow 1/f² behavior)')
plt.xlabel('Frequency [Hz]')
plt.ylabel('Power/Frequency [V**2/Hz]')
plt.legend()
plt.grid(True, alpha=0.3)
plt.show()


# 3

# Simulation parameters
dt = 0.01            # 100 Hz IMU
T = 20.0             # seconds
N = int(T / dt)

sigma_bg = 0.002     # bias random walk std (e.g. rad/s^2/sqrt(Hz))

# White noise driving the random walk
noise = sigma_bg * np.sqrt(dt) * np.random.randn(N)

# Random walk = cumulative sum
bias = np.zeros(N)
for k in range(1, N):
    bias[k] = bias[k-1] + noise[k]

time = np.arange(N) * dt

# Plot
plt.figure(figsize=(10, 4))
plt.plot(time, bias, label="Gyro bias random walk")
plt.xlabel("Time [s]")
plt.ylabel("Bias")
plt.title("IMU Bias Random Walk (Cumulative White Noise)")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
