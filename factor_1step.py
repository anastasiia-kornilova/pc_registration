import sys
import os
import glob
import open3d
import numpy as np
from tqdm import tqdm


from logger import get_configured_logger_by_name
logger = get_configured_logger_by_name(__file__)


voxel_size = 0.02
max_correspondence_distance_coarse = voxel_size * 15
max_correspondence_distance_fine = voxel_size * 1.5


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


def pairwise_registration(source, target):
    icp_coarse = open3d.registration.registration_icp(
        source, target, max_correspondence_distance_coarse, np.identity(4),
        open3d.registration.TransformationEstimationPointToPlane())
    icp_fine = open3d.registration.registration_icp(
        source, target, max_correspondence_distance_fine,
        icp_coarse.transformation,
        open3d.registration.TransformationEstimationPointToPlane())
    transformation_icp = icp_fine.transformation
    information_icp = open3d.registration.get_information_matrix_from_point_clouds(
        source, target, max_correspondence_distance_fine,
        icp_fine.transformation)
    return transformation_icp, information_icp


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

    for i in tqdm(range(len(pcds) - step_size - 1)):
        trans, _ = pairwise_registration(pcds[i + step_size], pcds[i])
        transformations.append(trans)

    np.save("{0}step_trans.npy".format(step_size), np.asarray(transformations))
