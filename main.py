import open3d
import numpy as np
import glob
import copy
import time
import numpy.linalg
import mrob


# Save pcd to png file. You have 5 seconds to rotate pcd for preferred position.
def save_pcd_to_png(name, pcd):
    vis = open3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.update_geometry()
    vis.poll_events()
    vis.update_renderer()
    vis.run()
    time.sleep(5)
    vis.capture_screen_image(name)
    vis.destroy_window()


def find_transformation(source, target, trans_init):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    threshold = 0.2
    source_temp.estimate_normals(search_param=open3d.geometry.KDTreeSearchParamHybrid(
        radius=0.5, max_nn=50))
    target_temp.estimate_normals(search_param=open3d.geometry.KDTreeSearchParamHybrid(
        radius=0.5, max_nn=50))
    transformation = open3d.registration.registration_icp(source_temp, target_temp, threshold
        , trans_init,open3d.registration.TransformationEstimationPointToPlane()).transformation
    return transformation


# To execute: main.py <directory to pcd-files>
if __name__ == '__main__':
    files = glob.glob('./pcds/*.pcd')
    print(len(files))
    files.sort()

    pcds = []
    print(files[0])
    for file in files:
        pcds.append(open3d.io.read_point_cloud(file))

    trans_sum_approximation = np.eye(4)
    pcd_full = [pcds[0]]
    for i in range(0, 304):
        # (0,70)(70,120)(120,165) -- good division

        trans = find_transformation(pcds[i + 1], pcds[i], np.eye(4))
        trans_sum_approximation = trans @ trans_sum_approximation
        res_trans = find_transformation(pcds[i + 1], pcd_full[-1], trans_sum_approximation)
        trans_sum_approximation = res_trans
        # Above we calculate approximation, error can be increased if we apply this transformation directly
        # Therefore the second estimation (below) for transformation is used
        # Also this approximation will be useful when skipping some frames
        if numpy.linalg.norm(mrob.SE3(trans).ln()) < 0.03:
            source = copy.deepcopy(pcds[i + 1]).transform(res_trans)
            pcd_full.append(source)

    open3d.visualization.draw_geometries(pcd_full)
