import open3d
import numpy as np
import glob
import copy
import time


# Save pcd to png file. You have 5 seconds to rotate pcd for preferred position.
def save_pcd_to_png(name, pcd):
    vis = open3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.update_geometry()
    vis.poll_events()
    vis.update_renderer()
    vis.run()
    time.sleep(3)
    vis.capture_screen_image(name)
    vis.destroy_window()


def draw_registration_result(source, target):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    threshold = 0.2
    trans_init = np.eye(4)
    source_temp.estimate_normals(search_param=open3d.geometry.KDTreeSearchParamHybrid(
        radius=0.5, max_nn=50))
    target_temp.estimate_normals(search_param=open3d.geometry.KDTreeSearchParamHybrid(
        radius=0.5, max_nn=50))
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    transformation = open3d.registration.registration_icp(source_temp, target_temp, threshold
                                                          , trans_init,open3d.registration.TransformationEstimationPointToPlane()).transformation
    return transformation


# To execute: main.py <directory to pcd-files>
if __name__ == '__main__':
    files = glob.glob('./pcds/*.pcd')
    print(len(files))
    files.sort()
    pcds = []
    for file in files:
        pcds.append(open3d.io.read_point_cloud(file))
    pcd_full = []
    source = pcds[0]
    trans_sum = np.eye(4)

    cnt = 50
    for i in range(0, cnt):
        print('{0} from {1}'.format(i, cnt))
        if i in []:
            break
        trans = draw_registration_result(pcds[i + 1], pcds[i])
        trans_sum = trans @ trans_sum
        source = copy.deepcopy(pcds[i + 1]).transform(trans_sum)
        pcd_full.append(source)

    open3d.visualization.draw_geometries(pcd_full)
