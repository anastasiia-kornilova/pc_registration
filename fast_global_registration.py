import numpy as np
import open3d
import open3d as o3d
import glob
import copy
import time
import numpy.linalg
import pickle

def load_obj(name):
    with open(name + '.pkl', 'rb') as f:
        return pickle.load(f)

def preprocess_point_cloud(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)

    #radius_normal = voxel_size * 2
    #pcd_down.estimate_normals(
    #    o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    
    # Point Feature Histograms
    radius_feature = voxel_size * 5
    pcd_fpfh = o3d.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh
    
def prepare_dataset(voxel_size, source, target):
    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh
    
def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size, result_ransac):
    distance_threshold = voxel_size * 0.4
    result = o3d.registration.registration_icp(
        source, target, distance_threshold, result_ransac.transformation,
        o3d.registration.TransformationEstimationPointToPlane())
    return result
    
def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.5
    result = o3d.registration.registration_fast_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result

if __name__ == '__main__':
    files = glob.glob('./2020-03-13-20-07_rectified/rectified/*.pcd')
    files.sort()

    pcds_outl = []
    for file in files:
        x = open3d.io.read_point_cloud(file)
        # remove outliers
        _, ind = x.remove_statistical_outlier(nb_neighbors=20, std_ratio=1)
        inlier_cloud = x.select_down_sample(ind)
        pcds_outl.append(inlier_cloud)

    rej = load_obj('./2020-03-13-20-07_rectified/results/rejected_poses')

    trans_sum_approximation = np.eye(4)
    pcd_full_glob = [pcds_outl[0]]
    s = 0
    voxel_size = 0.05

    for i in range(0, len(pcds_outl)-2):
        source, target, source_down, target_down, source_fpfh, target_fpfh = \
                prepare_dataset(voxel_size, pcds_outl[i + 1], pcds_outl[i])

        result_fast = execute_fast_global_registration(source_down, target_down,
                                                       source_fpfh, target_fpfh,
                                                       voxel_size)

        trans = refine_registration(source, target, source_fpfh, target_fpfh,
                                         voxel_size, result_fast)

        trans_sum_approximation = trans.transformation @ trans_sum_approximation
        res_trans = find_transformation(pcds_outl[i + 1], pcd_full_glob[-1], trans_sum_approximation)
        trans_sum_approximation = res_trans

        if s <= len(rej)-1 and i == rej[s]:
            s +=1
        else:
            source = copy.deepcopy(pcds_outl[i + 1]).transform(res_trans)
            pcd_full_glob.append(source)

    # open3d.visualization.draw_geometries(pcd_full_glob)
    
    # it seems that the good result is for 100:250
    open3d.visualization.draw_geometries(pcd_full_glob[100:250])
