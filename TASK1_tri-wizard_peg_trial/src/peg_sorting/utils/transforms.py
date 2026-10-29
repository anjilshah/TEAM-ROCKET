import numpy as np
from scipy.spatial.transform import Rotation


def wxyz(matrix):
    q = Rotation.from_matrix(matrix).as_quat()
    return q[[3, 0, 1, 2]]


def matrix_from_wxyz(q):
    return Rotation.from_quat(np.asarray(q)[[1, 2, 3, 0]]).as_matrix()


def downward_rotation(yaw=0.0):
    """Tool x points down; tool y is the jaw travel direction."""
    return np.array([[0, -np.sin(yaw), np.cos(yaw)],
                     [0, np.cos(yaw), np.sin(yaw)], [-1, 0, 0.]])


def symmetry_error(a, b, shape):
    period = 2 * np.pi / {"triangle": 3, "square": 4, "hexagon": 6, "circle": 1}[shape]
    if shape == "circle":
        return 0.0
    return float((a - b + period / 2) % period - period / 2)
