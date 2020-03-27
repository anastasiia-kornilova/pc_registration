import sys
import os
import glob
import open3d
import numpy as np
from tqdm import tqdm


from logger import get_configured_logger_by_name
logger = get_configured_logger_by_name(__file__)


def find_transformation(source, target, trans_init):
    threshold = 0.2
    if not source.has_normals():
        source.estimate_normals(search_param=open3d.geometry.KDTreeSearchParamHybrid(
            radius=0.5, max_nn=50))
    if not target.has_normals():
        target.estimate_normals(search_param=open3d.geometry.KDTreeSearchParamHybrid(
            radius=0.5, max_nn=50))
    transformation = open3d.registration.registration_icp(source, target, threshold
        , trans_init,open3d.registration.TransformationEstimationPointToPlane()).transformation
    return transformation


if __name__ == '__main__':
    pcds_dir = sys.argv[1]
    if not os.path.exists(pcds_dir):
        logger.error('Folder {0} for result does not exist.'.format())

    pcds_files = glob.glob(pcds_dir + '/*.pcd')
    pcds_files.sort()
    pcds = []
    for file in tqdm(pcds_files):
        pcds.append(open3d.io.read_point_cloud(file))
    logger.info('Read {0} PCDs from {1}'.format(len(pcds_files), pcds_dir))

    step_size = int(sys.argv[2])
    transformations = []
    if step_size == 2:
        for i in tqdm(range(len(pcds) - step_size)):
            trans1 = find_transformation(pcds[i + 1], pcds[i], np.eye(4))
            trans2 = find_transformation(pcds[i + 2], pcds[i + 1], np.eye(4))
            trans = find_transformation(pcds[i + 2], pcds[i], trans2 @ trans1)
            transformations.append(trans)
    else:
        for i in tqdm(range(len(pcds) - step_size)):
            trans = find_transformation(pcds[i + step_size], pcds[i], np.eye(4))
            transformations.append(trans)

    np.save("{0}step_trans.npy".format(step_size), np.asarray(transformations))
