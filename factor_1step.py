import sys
import os
import glob
import open3d
import numpy as np
from tqdm import tqdm
import pickle


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

def preprocess_point_cloud(pcd, voxel_size):
    # print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    # print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        open3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    # print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = open3d.registration.compute_fpfh_feature(
        pcd_down,
        open3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    # print(":: RANSAC registration on downsampled point clouds.")
    # print("   Since the downsampling voxel size is %.3f," % voxel_size)
    # print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = open3d.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
        open3d.registration.TransformationEstimationPointToPoint(False), 4, [
            open3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            open3d.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], open3d.registration.RANSACConvergenceCriteria(4000000, 500))
    return result


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


def pairwise_registration_ransac(source, target):
    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)

    result_ransac = execute_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                voxel_size)
    return result_ransac.transformation


def refine_registration(source, target, result_ransac, voxel_size=0.05):
    distance_threshold = voxel_size * 0.4
    # print(":: Point-to-plane ICP registration is applied on original point")
    # print("   clouds to refine the alignment. This time we use a strict")
    # print("   distance threshold %.3f." % distance_threshold)
    result = open3d.registration.registration_icp(
        source, target, distance_threshold, result_ransac,
        open3d.registration.TransformationEstimationPointToPlane())
    return result.transformation


if __name__ == '__main__':
    pcds_dir = sys.argv[1]
    if not os.path.exists(pcds_dir):
        logger.error('Folder {0} for result does not exist.'.format())

    pcds_files = glob.glob(pcds_dir + '/*.pcd')
    pcds_files.sort()
    pcds = []
    rej = pickle.load(open('2020-03-13-20-13-09_rectified2(1)/results/rejected_poses.pkl', 'rb'))
    print(rej)
    i = 0
    for file in tqdm(pcds_files):
        # if i not in rej:
        pcds.append(open3d.io.read_point_cloud(file))
        # i += 1
    logger.info('Read {0} PCDs from {1}'.format(len(pcds), pcds_dir))

    step_param = sys.argv[2]

    start_step, finish_step = None, None
    if '-' in step_param:
        start_step = int(step_param.split('-')[0])
        finish_step = int(step_param.split('-')[1])
    else:
        start_step = finish_step = int(step_param)

    print(start_step, finish_step)
    for step_size in range(start_step, finish_step + 1):
        transformations = []
        i = 0
        for i in tqdm(range(len(pcds) - step_size - 1)):
            trans, _ = pairwise_registration(pcds[i + step_size], pcds[i])
            # trans_result = refine_registration(pcds[i + step_size], pcds[i], trans_prel)
            transformations.append(trans)

        np.save("{0}step_trans.npy".format(step_size), np.asarray(transformations))
