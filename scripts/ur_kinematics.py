"""UR5e forward / inverse kinematics (numpy only).

FK uses the standard UR DH parameters. The URDF `base_link` is rotated 180 deg about z
relative to the DH base frame, which is applied here so results are in `base_link`
(== `world` in this simulation) and match `ros2 run tf2_ros tf2_echo world tool0`.
"""
import numpy as np

# UR5e DH parameters (https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/)
D = [0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996]
A = [0.0, -0.425, -0.3922, 0.0, 0.0, 0.0]
ALPHA = [np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0]

# base_link -> DH base
BASE = np.diag([-1.0, -1.0, 1.0, 1.0])

# tool0 -> gripper_tcp (see urdf/parallel_gripper.xacro)
TCP_OFFSET = 0.08

JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]


def _dh(theta, d, a, alpha):
    ct, st, ca, sa = np.cos(theta), np.sin(theta), np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ])


def fk(q, tcp=True):
    """4x4 pose of gripper_tcp (or tool0 if tcp=False) in base_link."""
    T = BASE.copy()
    for i in range(6):
        T = T @ _dh(q[i], D[i], A[i], ALPHA[i])
    if tcp:
        T = T @ np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, TCP_OFFSET], [0, 0, 0, 1.0]])
    return T


def rot_z_down(yaw):
    """Rotation of a tool pointing straight down (tool z = -world z), rotated by yaw about world z."""
    c, s = np.cos(yaw), np.sin(yaw)
    # x axis in the horizontal plane, z axis down, y = z cross x
    x = np.array([c, s, 0.0])
    z = np.array([0.0, 0.0, -1.0])
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def _pose_error(T, target):
    dp = target[:3, 3] - T[:3, 3]
    R = target[:3, :3] @ T[:3, :3].T
    # axis-angle (small-angle form is enough near the solution, exact form far from it)
    angle = np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    if angle < 1e-9:
        dr = np.zeros(3)
    else:
        axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * np.sin(angle))
        dr = axis * angle
    return np.concatenate([dp, dr])


def ik(target, q_seed, iters=200, tol=1e-5):
    """Damped least-squares IK for the TCP. Returns joint angles close to q_seed, or None."""
    q = np.array(q_seed, dtype=float)
    eps = 1e-6
    for _ in range(iters):
        err = _pose_error(fk(q), target)
        if np.linalg.norm(err) < tol:
            return q
        J = np.zeros((6, 6))
        for j in range(6):
            dq = np.zeros(6)
            dq[j] = eps
            J[:, j] = _pose_error(fk(q), fk(q + dq)) / eps
        lam = 0.01
        q += J.T @ np.linalg.solve(J @ J.T + lam * np.eye(6), err)
    return None
