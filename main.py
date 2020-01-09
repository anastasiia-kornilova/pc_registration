import open3d
import numpy as np
import glob
import copy
import time
from itertools import chain


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


def find_transfomation(source, target, trans_init):
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
    print(pcds[0].has_normals())
    print(pcds[0].has_colors())
    source = pcds[0]
    trans_sum_approximation = np.eye(4)

    pcd_full = [pcds[0]]
    for i in range(0, 70):
        # if i in chain(range(51, 95), range(105, 120)):
        #     print(i)
        #     continue

        trans = find_transfomation(pcds[i + 1], pcds[i], np.eye(4))
        trans_sum_approximation = trans @ trans_sum_approximation
        # Above we calculate approximation, error can be increased if we apply this transformation directly
        # Therefore the second estimation (below) for transformation is used
        res_trans = find_transfomation(pcds[i + 1], pcd_full[i], trans_sum_approximation)
        source = copy.deepcopy(pcds[i + 1]).transform(res_trans)
        pcd_full.append(source)

    open3d.visualization.draw_geometries(pcd_full)
