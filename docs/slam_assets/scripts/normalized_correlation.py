import numpy as np

def normalized_correlation(x, y, eps=1e-8):
    x = x.astype(np.float64)
    y = y.astype(np.float64)

    mu_x = np.mean(x)
    mu_y = np.mean(y)
    sigma_x = np.std(x)
    sigma_y = np.std(y)
    covariance = np.mean((x - mu_x) * (y - mu_y))

    return covariance / (sigma_x * sigma_y + eps)

# Example
x = np.array([1, 2, 3, 4, 5])
y = np.array([2, 3, 4, 5, 6])  # same pattern, just shifted up
print(normalized_correlation(x, y))  # ~1.0 (perfect correlation)
