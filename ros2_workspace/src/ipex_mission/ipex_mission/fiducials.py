"""
Uses detected tags to estimate the rover base_link pose. By knowing fiducials positions, it's possible
to estimate the detecting camera pose. Then via camera -> base_link transform, base_link pose is known.
"""
import math
import cv2
import numpy as np

# used to convert from opencv order to apriltag order
APRILTAG_TO_SURVEY = (1, 0, 3, 2)


class TagMap:
    def __init__(self, info: dict) -> None:
        self.tags: dict[int, np.ndarray] = {int(tag["id"]): np.asarray(tag["corners"], dtype=np.float64) for tag in info["tags"]}

    def center(self, tag_id):
        return self.tags[tag_id].mean(axis=0)


def survey_order(apriltag_corners):
    """The 4 image corners of an apriltag_msgs/AprilTagDetection as a (4, 2) array in the survey order."""
    pts = np.array([[c.x, c.y] for c in apriltag_corners], dtype=np.float64)
    return pts[list(APRILTAG_TO_SURVEY)]


def coplanar(pts, tol=0.01):
    """Checks if all pts lie on the same plane within tol."""
    n = np.cross(pts[1] - pts[0], pts[3] - pts[0])
    n = n / np.linalg.norm(n)
    return np.abs((pts - pts[0]) @ n).max() < tol


def camera_pose(tag_map, detections, k_matrix, distortion=None):
    """Executes PnP on all detected corners. Returns translation and rotation matrixes for camera transform."""
    if not detections:
        return None
    
    obj = np.concatenate([tag_map.tags[i] for i in sorted(detections)])
    img = np.concatenate([detections[i] for i in sorted(detections)])
    k_matrix = np.asarray(k_matrix, dtype=np.float64).reshape(3, 3)
    distortion = np.zeros(5) if distortion is None else np.asarray(distortion, dtype=np.float64)
    
    # tags on one plane: IPPE; on several planes: SQPnP
    flag = cv2.SOLVEPNP_IPPE if coplanar(obj) else cv2.SOLVEPNP_SQPNP
    ok, rvec, tvec = cv2.solvePnP(obj, img, k_matrix, distortion, flags=flag)

    if not ok:
        return None
    
    rvec, tvec = cv2.solvePnPRefineLM(obj, img, k_matrix, distortion, rvec, tvec)

    if not (np.all(np.isfinite(rvec)) and np.all(np.isfinite(tvec))):
        return None
    
    proj, _ = cv2.projectPoints(obj, rvec, tvec, k_matrix, distortion)
    rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1))))
    R_cm, _ = cv2.Rodrigues(rvec)
    R_mc = R_cm.T
    t_mc = -R_mc @ tvec.ravel()
    return R_mc, t_mc, rms


def pose_ambiguity(tag_map, detections, k_matrix, distortion=None):
    """Coplanar tags return two possible poses. If best_rms / second_best_rms is circa 1, then both poses fit."""
    obj = np.concatenate([tag_map.tags[i] for i in sorted(detections)])
    if not coplanar(obj):
        return 0.0
    
    img = np.concatenate([detections[i] for i in sorted(detections)])
    k_matrix = np.asarray(k_matrix, dtype=np.float64).reshape(3, 3)
    distortion = np.zeros(5) if distortion is None else np.asarray(distortion, dtype=np.float64)
    n, _, _, err = cv2.solvePnPGeneric(obj, img, k_matrix, distortion, flags=cv2.SOLVEPNP_IPPE)

    if n < 2:
        return 0.0
    e = np.sort(np.asarray(err, dtype=np.float64).ravel())

    return float(e[0] / e[1]) if e[1] > 1e-12 else 1.0


def base_pose(R_mc, t_mc, R_bc, t_bc):
    """Applies translationa and rotation matrix to obtain rover's base_link pose"""
    R_mb = R_mc @ R_bc.T
    t_mb = t_mc - R_mb @ t_bc
    yaw = math.atan2(R_mb[1, 0], R_mb[0, 0])
    return t_mb[0], t_mb[1], t_mb[2], yaw

# https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
def quat_to_rot(x, y, z, w):
    """Converts a quaternion to a rotation matrix."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
