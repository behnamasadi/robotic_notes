import numpy as np
from liegroups import SE3


xi = np.array([0.1, 0.0, 0.0, 0.01, 0.02, 0.03])  # [rho, phi]
T = SE3.exp(xi)
