# тулы


import shutil # library to delete directory recursevly
import os
import pickle
import open3d
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches # for manual legend

def create_folder(outFolder):
    """
    Function create folder at path_folder. If folder already exist, that folder will be deletet recursevly and created new one
    :param path_folder:  string of folder
    """
    try:
        os.mkdir(outFolder)
    except Exception:
        print("Folder already exist. Removing contents...")
        shutil.rmtree(outFolder)
        os.mkdir(outFolder) # create folder again


def save_obj(obj, name):
    """
    Save numpy array in pickle format
    :param obj:
    :param name:
    :return:
    """
    with open(name + '.pkl', 'wb') as f:
        pickle.dump(obj, f, pickle.HIGHEST_PROTOCOL)

def load_obj(name ):
    """
    Load numpy array in pickle format
    :param name:
    :return:
    """
    with open(name + '.pkl', 'rb') as f:
        return pickle.load(f)


def list2PCD(list_pcds):
    """
    Convert list of pcd open3d objects to one pcd object
    :param list_pcds: list
    :return: open3d object
    """
    aggregated_pcd = np.zeros((0, 3))
    for pcd in list_pcds:
        aggregated_pcd = np.vstack((aggregated_pcd, np.asarray(pcd.points)))
    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(aggregated_pcd)
    return pcd


def plot_pose(frames=[], poses=[], black_poses=[], show_t=True, show_origin_frame=True, show_coord=True, show_pose=True, show_vector=False,  t_names=['$\mathbf{t}$']):
    """Plotting poses in frames

    Args:
        frames (list): frames to plot in addition to original frame.
        poses (list): poses to plot in the original frame.

        show_t (bool): depict t-vector name
        show_coord (bool): depict pose coordinates

    Returns:
        None.
    """

    def get_xyzuvctt(T):
        t = T[:3, 3:4].flatten()
        t_ = t / (np.linalg.norm(t) + 1e-5) * 1
        R = T[:3, :3]/5

        (x, y, z), (u, v, c) = np.repeat(t.reshape(3, 1), 3, axis=1), R
        return x, y, z, u, v, c, t, t_

    fig = plt.figure(figsize=(5, 5))
    ax = fig.add_subplot(111, projection='3d')
    lngth = 0.5

    # fig appearance
    ax.view_init(elev=22, azim=32)
    # ax.set_aspect('equal')
    ax.view_init(elev=22, azim=20)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    # ax.axis('off');  # ax.axis('equal')

    if show_origin_frame:
        # origin and coordinate axes (coordinate frame)
        xl = 'x';
        yl = 'y';
        zl = 'z';
        ol = 'O'
        x, y, z, u, v, c, t, t_ = get_xyzuvctt(np.eye(4))

        ax.scatter(t_[0], t_[1], t_[2], s=80, c='k')
        ax.quiver(x, y, z, u, v, c, color='k', length=lngth, arrow_length_ratio=0.2, linewidth=0.5)
        ax.text(0, -0.5, -0.1, ol);
        ax.text(lngth * 1.2, 0, 0, xl);
        ax.text(0, lngth * 1.2, 0, yl);
        ax.text(0, 0, lngth * 1.2, zl)
        # aid to have initial minimum dimensions of plot
    ax.scatter(1.5, 1.5, 1.5, alpha=0)

    for frame in frames:
        x, y, z, u, v, c, t, t_ = get_xyzuvctt(frame)
        ax.scatter(t_[0], t_[1], t_[2], s=80, c='k')
        ax.quiver(
            x, y, z, u, v, c, color='k', length=lngth, arrow_length_ratio=0.2, linewidth=0.5)
        xl += '\''
        yl += '\''
        zl += '\''
        ol += '\''
        ax.text(t_[0], t_[1] - 0.5, t_[2] - 0.1, ol);
        frame = frame @ np.diag([lngth * 1.2, lngth * 1.2, lngth * 1.2, 1])
        ax.text(frame[:3, :3][0][0] + t_[0], frame[:3, :3][1][0] + t_[1], frame[:3, :3][2][0] + t_[2], xl);
        ax.text(frame[:3, :3][0][1] + t_[0], frame[:3, :3][1][1] + t_[1], frame[:3, :3][2][1] + t_[2], yl);
        ax.text(frame[:3, :3][0][2] + t_[0], frame[:3, :3][1][2] + t_[1], frame[:3, :3][2][2] + t_[2], zl);

    for i, pose in enumerate(poses):
        # given frame
        x, y, z, u, v, c, t, t_ = get_xyzuvctt(pose)
        if show_pose:
            color = ['r', 'g', 'b']
            if i in black_poses:
                color = ["black", "black", "black"]
            ax.quiver(x, y, z, u, v, c, arrow_length_ratio=0, color=color, linestyle='-', linewidth=4)
            ax.text(t[0], t[1], t[2], str(i));
        if show_vector:
            alr = 1 / (np.linalg.norm(t_) + 1e-5) * 0.1
            ax.quiver(0, 0, 0, t_[0], t_[1], t_[2], color='k', arrow_length_ratio=alr, linewidth=2)
            for frame in frames:
                _, _, _, _, _, _, _, tf_ = get_xyzuvctt(frame)
                tf2_ = t_ - tf_
                ax.quiver(tf_[0], tf_[1], tf_[2], tf2_[0], tf2_[1], tf2_[2],
                          color='k', arrow_length_ratio=alr, linewidth=2)
        # t-vector name
        # if show_t:
        # tf2_ = t_-tf_/2
        #    ax.text(t_[0]/2, t_[1]/2, t_[2]/2+0.4, t_names[i], fontsize=20)
        # coordinates
        if show_coord:
            ax.text(t_[0], t_[1], t_[2] - 0.7, '(' + str(t[0]) + ',' + str(t[1]) + ',' + str(t[2]) + ')')


if __name__ == "__main__":
    path = r"data/moving_formatted/results/rejected_poses"
    x = load_obj(path)
    print(x)
    path = r"data/moving_formatted/results/3DPoses"
    x = load_obj(path)
    print(x)
    print(len(x))

