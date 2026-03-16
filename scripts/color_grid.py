from collections import namedtuple  # noqa: D100
from math import cos, sin

import numpy as np

ColorGrid = namedtuple("ColorGrid", ["positions", "colors"])


def build_color_grid(x_count: int = 10, y_count: int = 10, z_count: int = 10, twist: int = 0) -> ColorGrid:
    """
    Create a cube of points with colors.

    The total point cloud will have x_count * y_count * z_count points.

    Parameters
    ----------
    x_count, y_count, z_count:
        Number of points in each dimension.
    twist:
        Angle to twist from bottom to top of the cube

    """
    grid = np.mgrid[
        slice(-x_count, x_count, x_count * 1j),
        slice(-y_count, y_count, y_count * 1j),
        slice(-z_count, z_count, z_count * 1j),
    ]

    angle = np.linspace(-float(twist) / 2, float(twist) / 2, z_count)
    for z in range(z_count):
        xv, yv, zv = grid[:, :, :, z]
        rot_xv = xv * cos(angle[z]) - yv * sin(angle[z])
        rot_yv = xv * sin(angle[z]) + yv * cos(angle[z])
        grid[:, :, :, z] = [rot_xv, rot_yv, zv]

    positions = np.vstack([xyz.ravel() for xyz in grid])

    colors = np.vstack([
        xyz.ravel()
        for xyz in np.mgrid[
            slice(0, 255, x_count * 1j),
            slice(0, 255, y_count * 1j),
            slice(0, 255, z_count * 1j),
        ]
    ])

    return ColorGrid(positions.T, colors.T.astype(np.uint8))