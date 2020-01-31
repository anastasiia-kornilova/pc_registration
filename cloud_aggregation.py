import numpy as np
import mrob
import copy
import math
import open3d
import glob
from main import find_transformation


def rectify(source, transformation, discr_count=10):
    timestamp_split = split_by_timestamp(source)
    discretization_splited = split_in_groups(timestamp_split, discr_count)
    aggregated_cloud = rectify_groups(discretization_splited, transformation)
    return aggregated_cloud


def rectify_groups(source_groups, transformation):
    discr_cnt = len(source_groups)
    xi_ini = np.array([0, 0, 0, 0, 0, 0], dtype='float64')
    xi_fin = mrob.SE3(transformation).ln()
    xi = np.zeros((discr_cnt, 6))
    for i in range(6):
        xi[:, i] = np.linspace(xi_ini[i], xi_fin[i], discr_cnt, dtype='float64')
    res = []
    for i in range(discr_cnt):
        new_one = copy.deepcopy(source_groups[i]).transform(mrob.SE3(xi[i, :]).inv().T())
        res.append(new_one.paint_uniform_color([1, 0.706, 0]))

    return res


def split_in_groups(timestamps_split, discr_cnt = 10):
    ln = math.ceil(len(timestamps_split) / discr_cnt)
    pcds = []
    for i in range(discr_cnt):
        pcd = open3d.geometry.PointCloud()
        xyz = []
        for j in range(ln):
            ind = i * ln + j
            print(ind, len(timestamps_split))
            if ind < len(timestamps_split):
                group = timestamps_split[ind]
                for k in range(len(group)):
                    xyz.append(group[k])
        arr = np.array(xyz)
        # There maybe some problems with the last array, therefore check is needed
        if len(arr) != 0:
            print(arr)
            pcd.points = open3d.utility.Vector3dVector(arr)
            pcds.append(pcd.paint_uniform_color([0, 0, 1]))
    return pcds


def split_by_timestamp(source):
    colors = np.asarray(source.colors)
    points = np.asarray(source.points)
    pcd_split = []
    new_group = []
    prev_color = colors[0]
    for i in range(len(colors)):
        curr_color = colors[i]
        eq = True
        for j in range(3):
            if curr_color[j] != prev_color[j]:
                eq = False
                break
        if eq:
            new_group.append(points[i])
        else:
            pcd_split.append(new_group.copy())
            new_group = []

        prev_color = curr_color

    return pcd_split


if __name__ == '__main__':
    files = glob.glob('./timestamps/*.pcd')
    print(len(files))
    files.sort()
    files = files[:155]

    pcds = []
    for file in files:
        pcds.append(open3d.io.read_point_cloud(file))

    transformation = find_transformation(pcds[150], pcds[151], np.eye(4))

    timestamp_split = split_by_timestamp(pcds[150])
    discretization_splited = split_in_groups(timestamp_split, discr_cnt=20)
    aggregated_cloud = rectify_groups(discretization_splited, transformation)
    open3d.visualization.draw_geometries(discretization_splited + aggregated_cloud)
