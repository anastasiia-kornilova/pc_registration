import open3d as o3d
import numpy as np
import copy
import os
path = r'mrob/lib'
import sys
sys.path.append(path)
import mrob
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches # for manual legend
from cloud_aggregation import *
from scipy.spatial.transform import Rotation as R
from tools import *
from filters import *
import pickle as pkl
import time
# @title


def find_transformation(source, target, trans_init):
    threshold = 0.2
    if not source.has_normals():
        source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=50))
    if not target.has_normals():
        target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=50))
    transformation = o3d.registration.registration_icp(source, target, threshold, trans_init,
                                                          o3d.registration.TransformationEstimationPointToPlane()).transformation
    return transformation


# specified relative pathes to raw point clouds. Unccoment required raw data
# path = r"data/2020-01-21/formatted"
# path = r"data/2020-03-13-20-07/formatted"
# path = r"data/2020-03-13-20-13-09/formatted"
path = r"data/2020-03-13-20-14-16/formatted"


if __name__ == "__main__":
    d = 0.2 # heuristic
    files = []
    path_out_rectified_pcds = os.path.join(os.path.join(os.getcwd(), os.path.dirname(path)), "rectified")
    path_out_results = os.path.join(os.path.join(os.getcwd(), os.path.dirname(path)), "results")
    create_folder(path_out_rectified_pcds) #folder for rectified point clouds
    create_folder(path_out_results) # folder for results (trajectories in the form of 4x4 matrices)


    for (dirpath, dirnames, filenames) in os.walk(path):
        for file in filenames:
            files.append(os.path.join(dirpath, file))


    print("Number of pcds founded:", len(files))
    files.sort()
    print(files)

    # files = files[:590]

    pcds = [] # list of rectified pcds
    for file in files:
        pcd = o3d.io.read_point_cloud(file)
        # pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2)
        if pcd.is_empty():
            continue
        pcds.append(pcd)

    trans_sum_approximation = np.eye(4)
    pcd_full = [copy.deepcopy(rectify(pcds[0], trans_sum_approximation))]
    cnt = 0

    poses = [trans_sum_approximation] # 3d poses
    rejected_poses = [] # list of indices of poses, which were removed
    iPose = 0 # last accepted pose
    lin_transl = []
    eulers = np.zeros((0, 3))
    elapsed_times = []

    for i in range(0, 10):
        s = time.time()
        trans = find_transformation(pcds[i + 1], pcds[i], np.eye(4)) # local transformation btwn pcds
        pcd = rectify(pcds[i + 1], np.linalg.inv(trans))
        pcds[i + 1] = copy.deepcopy(pcd)
        res_trans = find_transformation(pcd, pcd_full[-1], trans_sum_approximation @ np.linalg.inv(trans))
        trans_sum_approximation = res_trans

        tm = R.from_matrix(trans[:3,:3])
        euler = tm.as_euler('zyx', degrees=True)
        eulers = np.vstack((eulers, euler))
        lin_transl.append(np.linalg.norm(mrob.SE3(trans).ln()))

        if filter(lin_transl[-1], iPose, trans_sum_approximation, poses, d=d):
            print(i)
            iPose = i
            source = copy.deepcopy(pcd).transform(res_trans)
            pcd_full.append(source)
        else:
            rejected_poses.append(i) # add reh=jected poses
        cnt += 1
        poses.append(trans_sum_approximation)
        elapsed_times.append((time.time() - s) * 1000)

    plot_pose(poses=poses,black_poses=rejected_poses, show_coord=False, show_vector=False, show_origin_frame=False)
    plt.draw()
    plt.savefig(os.path.join(path_out_results, "3D Trajectory.png"))
    plt.show(block=True)


    # draw 2d plot with text along trajectory
    fig, axes = plt.subplots(nrows=2, ncols=2)
    fig.set_figheight(6)
    fig.set_figwidth(18)
    for j, ax in enumerate([axes[0,0], axes[1,0]]):
        for i, pose in enumerate(poses):
            color = "g"
            if i in rejected_poses:
                color = "black"
            ax.scatter(pose[0, 3], pose[1, 3], c=color)
            if j == 0:
                if color == "black" or i % 3 == 0:
                    ax.text((pose[0, 3] + 0.001), (pose[1, 3] + 0.001), str(i), fontsize=8)
                ax.set_title("Numbered trajectory XOY")
            else:
                ax.set_title("Trajectory XOY")
        ax.set_ylabel("Y, [meters]")
        ax.set_xlabel("X, [meters]")
        accepted = mpatches.Patch(color='green', label='Accepted poses')
        dropped = mpatches.Patch(color='black', label='Dropped poses')
        ax.legend(handles=[accepted, dropped])

    # draw eulers plot
    ax = axes[0,1]
    ax.plot(eulers[:, 0], label="Rz")
    ax.plot(eulers[:, 1], label="Ry")
    ax.plot(eulers[:, 2], label="Rx")
    ax.scatter(rejected_poses, eulers[rejected_poses][:, 0], label="Dropped", color="black")
    ax.scatter(rejected_poses, eulers[rejected_poses][:, 1], color="black")
    ax.scatter(rejected_poses, eulers[rejected_poses][:, 2], color="black")
    ax.set_title("Eulers between frames")
    ax.set_ylabel("Degrees")
    ax.set_xlabel("Iteration")
    ax.legend()

    #draw linear translation
    ax = axes[1,1]
    ax.plot(lin_transl, label="translation")
    rej = [lin_transl[i] for i in rejected_poses]
    ax.scatter(rejected_poses, rej, label="dropped", color="black")
    ax.set_title(f"Transition vector norm. Heuristic < {d} meters")
    ax.set_ylabel("$||translation||_2$")
    ax.set_xlabel("Iteration")
    ax.legend()

    plt.subplots_adjust(hspace=0.5)
    plt.draw()
    plt.savefig(os.path.join(path_out_results, "Plots"))
    plt.show(block=True)


    fig = plt.figure(1)
    plt.plot(elapsed_times)
    total_time = "{0:.2f}".format(sum(elapsed_times)/1000)
    plt.title(f"Elapsed times. Total time: {total_time} sec.")
    plt.ylabel("Time, [ms]")
    plt.xlabel("Iteration")
    plt.draw()
    plt.savefig(os.path.join(path_out_results, "Elapsed_time"))
    plt.show(block=True)

    print(cnt)
    o3d.visualization.draw_geometries(pcd_full)
    o3d.io.write_point_cloud(os.path.join(path_out_results, f'ICP_PCD.pcd'), list2PCD(pcd_full), write_ascii=True)

    # Saving all required arrays
    save_obj(np.asarray(poses), os.path.join(path_out_results, "3DPoses"))
    print(f"Saved {len(poses)} poses")
    save_obj(eulers, os.path.join(path_out_results, "eulers"))
    save_obj(np.asarray(rejected_poses), os.path.join(path_out_results, "rejected_poses"))

    for i, pcd in enumerate(pcds[:cnt + 1]):
        o3d.io.write_point_cloud(os.path.join(path_out_rectified_pcds, f'{i:03d}.pcd'), pcds[i], write_ascii=True)

