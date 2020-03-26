path = r'mrob/lib'
import sys
sys.path.append(path)
import mrob
import numpy as np


def filter(lin_transl, iPose, curr_T, poses, d=0.2):
    """
    Heuristic filter to remove outliers in trajectory.
    That filter consists of 2 checks:
    - measure  norm between to last poses, that should be less than d,
    - measure norm between current and last accepted pose. Should be less than d * (i - iPose) * 1.2

    :param lin_transl: float, norm between last 2 poses
    :param iPose: int,  index of last accepted position
    :param curr_T: np.ndarray 4x4,  current transformation matrix of the pose
    :param poses: list, all poses
    :param d: float, heuristic limit of the translation
    :return: bool, outlier or not current
    """
    i = len(poses) - 1 # current index
    T_trans = poses[iPose] @ np.linalg.inv(curr_T)  # transition matrix from last unfiltered to current position
    dist_last_accepted = np.linalg.norm(mrob.SE3(T_trans).ln())  # distance from last unfiltered position to current
    # debug
    print(i, iPose, (lin_transl < d) and (dist_last_accepted < (i - iPose) * 0.205), lin_transl, dist_last_accepted)
    return ((lin_transl < d) and (dist_last_accepted < (i - iPose) * 0.205))
