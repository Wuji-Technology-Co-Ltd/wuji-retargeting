from __future__ import annotations

import numpy as np


def normalize_quat_xyzw(q) -> np.ndarray:
    quat = np.asarray(q, dtype=float)
    if quat.shape != (4,) or not np.isfinite(quat).all():
        raise ValueError("quaternion must contain 4 finite values")
    norm = np.linalg.norm(quat)
    if norm <= 1e-12 or not np.isfinite(norm):
        raise ValueError("quaternion must be finite and non-zero")
    return quat / norm


def quat_multiply_xyzw(a, b) -> np.ndarray:
    ax, ay, az, aw = normalize_quat_xyzw(a)
    bx, by, bz, bw = normalize_quat_xyzw(b)
    return normalize_quat_xyzw(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ]
    )


def quat_inverse_xyzw(q) -> np.ndarray:
    x, y, z, w = normalize_quat_xyzw(q)
    return np.asarray([-x, -y, -z, w], dtype=float)


def quat_to_matrix_xyzw(q) -> np.ndarray:
    x, y, z, w = normalize_quat_xyzw(q)
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def matrix_to_quat_xyzw(matrix) -> np.ndarray:
    rotation = np.asarray(matrix, dtype=float)
    if rotation.shape != (3, 3) or not np.isfinite(rotation).all():
        raise ValueError("rotation matrix must be 3x3 and finite")
    u, _, vh = np.linalg.svd(rotation)
    rotation = u @ vh
    if np.linalg.det(rotation) < 0.0:
        u[:, -1] *= -1.0
        rotation = u @ vh
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        quat = [(rotation[2, 1] - rotation[1, 2]) / scale, (rotation[0, 2] - rotation[2, 0]) / scale, (rotation[1, 0] - rotation[0, 1]) / scale, 0.25 * scale]
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            quat = [0.25 * scale, (rotation[0, 1] + rotation[1, 0]) / scale, (rotation[0, 2] + rotation[2, 0]) / scale, (rotation[2, 1] - rotation[1, 2]) / scale]
        elif index == 1:
            scale = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            quat = [(rotation[0, 1] + rotation[1, 0]) / scale, 0.25 * scale, (rotation[1, 2] + rotation[2, 1]) / scale, (rotation[0, 2] - rotation[2, 0]) / scale]
        else:
            scale = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            quat = [(rotation[0, 2] + rotation[2, 0]) / scale, (rotation[1, 2] + rotation[2, 1]) / scale, 0.25 * scale, (rotation[1, 0] - rotation[0, 1]) / scale]
    return normalize_quat_xyzw(quat)


def align_quat_sign_xyzw(q, reference) -> np.ndarray:
    quat = normalize_quat_xyzw(q)
    reference_quat = normalize_quat_xyzw(reference)
    return -quat if float(np.dot(quat, reference_quat)) < 0.0 else quat


def quat_angle_xyzw(a, b) -> float:
    dot = abs(float(np.dot(normalize_quat_xyzw(a), normalize_quat_xyzw(b))))
    return 2.0 * float(np.arccos(np.clip(dot, -1.0, 1.0)))


def quat_slerp_xyzw(a, b, fraction: float) -> np.ndarray:
    start = normalize_quat_xyzw(a)
    end = align_quat_sign_xyzw(b, start)
    amount = float(np.clip(fraction, 0.0, 1.0))
    dot = float(np.clip(np.dot(start, end), -1.0, 1.0))
    if dot > 0.9995:
        return normalize_quat_xyzw(start + amount * (end - start))
    angle = float(np.arccos(dot))
    sin_angle = float(np.sin(angle))
    return normalize_quat_xyzw(
        np.sin((1.0 - amount) * angle) / sin_angle * start
        + np.sin(amount * angle) / sin_angle * end
    )


def scale_quat_rotation_xyzw(q, scale: float) -> np.ndarray:
    return quat_slerp_xyzw([0.0, 0.0, 0.0, 1.0], q, float(scale))


def mean_quaternion_xyzw(quaternions) -> np.ndarray:
    values = np.asarray([normalize_quat_xyzw(q) for q in quaternions], dtype=float)
    if values.ndim != 2 or values.shape[0] == 0:
        raise ValueError("at least one quaternion is required")
    reference = values[0]
    values = np.asarray([align_quat_sign_xyzw(q, reference) for q in values])
    eigenvalues, eigenvectors = np.linalg.eigh(values.T @ values)
    return align_quat_sign_xyzw(eigenvectors[:, int(np.argmax(eigenvalues))], reference)


def validate_rotation_matrix(matrix, atol: float = 1e-6) -> np.ndarray:
    rotation = np.asarray(matrix, dtype=float)
    if rotation.shape != (3, 3) or not np.isfinite(rotation).all():
        raise ValueError("axis transform must be a finite 3x3 matrix")
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=atol):
        raise ValueError("axis transform must be orthogonal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=atol):
        raise ValueError("axis transform determinant must be +1")
    return rotation


def compose_pose(position_a, orientation_a, position_b, orientation_b) -> tuple[np.ndarray, np.ndarray]:
    position = np.asarray(position_a, dtype=float) + quat_to_matrix_xyzw(orientation_a) @ np.asarray(position_b, dtype=float)
    orientation = quat_multiply_xyzw(orientation_a, orientation_b)
    return position, orientation


def inverse_pose(position, orientation) -> tuple[np.ndarray, np.ndarray]:
    inverse_orientation = quat_inverse_xyzw(orientation)
    inverse_position = -(quat_to_matrix_xyzw(inverse_orientation) @ np.asarray(position, dtype=float))
    return inverse_position, inverse_orientation


def transform_point_inverse(position, orientation, point) -> np.ndarray:
    return quat_to_matrix_xyzw(orientation).T @ (np.asarray(point, dtype=float) - np.asarray(position, dtype=float))


def yaw_from_quat_y_up(q) -> float:
    rotation = quat_to_matrix_xyzw(q)
    return float(np.arctan2(rotation[0, 2], rotation[2, 2]))


def quat_from_yaw_y_up(yaw: float) -> np.ndarray:
    half = float(yaw) * 0.5
    return normalize_quat_xyzw([0.0, np.sin(half), 0.0, np.cos(half)])
