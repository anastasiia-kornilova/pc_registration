import numpy as np
import sys
import glob
import os
from tqdm import tqdm
import open3d
import logger
import copy
import mrob

from logger import get_configured_logger_by_name
logger = get_configured_logger_by_name(__file__)

if __name__ == '__main__':
    traj = np.load("traj.npy")
    print(traj.shape)

    pcds_dir = sys.argv[1]
    if not os.path.exists(pcds_dir):
        logger.error('Folder {0} for result does not exist.'.format())

    pcds_files = glob.glob(pcds_dir + '/*.pcd')
    pcds_files.sort()
    pcds = []
    for file in tqdm(pcds_files):
        pcds.append(open3d.io.read_point_cloud(file))
    logger.info('Read {0} PCDs from {1}'.format(len(pcds_files), pcds_dir))

    pcd_full = [pcds[0]]
    for i in tqdm(range(traj.shape[0] - 2)):
        source = copy.deepcopy(pcds[i + 1]).transform((mrob.SE3(traj[i + 1]).T()))
        pcd_full.append(source)

    open3d.visualization.draw_geometries(pcd_full)
