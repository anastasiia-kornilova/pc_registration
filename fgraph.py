import mrob
import numpy as np
import glob
import open3d
import matplotlib.pyplot as plt
from tqdm import tqdm


def print_3d_graph(graph):
    '''This function draws the state variables for a 3D pose graph'''

    # read graph, returns a list (vector) of state (np arrays)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    x = graph.get_estimated_state()
    To = mrob.SE3(np.zeros(6)) #XXX for some reason (ownership?) can't do this is 1 line
    prev_p =To.T()[:3, 3]
    for xi in x:
        Ti = mrob.SE3(xi)
        p = Ti.T()[:3,3]
        ax.plot((prev_p[0],p[0]),(prev_p[1],p[1]),(prev_p[2],p[2]) , '-b')
        prev_p = np.copy(p)
    plt.show()


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
    files = glob.glob('./pcds/*.pcd')
    print(len(files))
    files.sort()
    files = files[:3]

    pcds = []
    print(files[0])
    for file in files:
        pcds.append(open3d.io.read_point_cloud(file))

    invCov = np.identity(6)
    graph = mrob.FGraph()
    x1 = np.zeros(6)
    # First pose
    anchor = graph.add_node_pose_3d(x1)
    # Anchor pose
    graph.add_factor_1pose_3d(np.zeros(6), anchor, invCov * 1e6)

    vertex_list = [anchor]
    step1_factors = np.load("1step_trans.npy")
    print(step1_factors.shape)
    last_id = anchor
    pos = mrob.SE3(x1).T()
    for i in tqdm(range(step1_factors.shape[0])):
        trans = step1_factors[i]
        pos = trans @ pos
        pos_ln = mrob.SE3(pos).ln()
        new_id = graph.add_node_pose_3d(pos_ln)
        vertex_list.append(new_id)
        graph.add_factor_2poses_3d(mrob.SE3(trans).ln(), last_id, new_id, invCov * 1e4)
        last_id = new_id

    step2_factors = np.load("2step_trans.npy")
    for i in tqdm(range(step2_factors.shape[0])):
        trans = step1_factors[i]
        graph.add_factor_2poses_3d(mrob.SE3(trans).ln(), vertex_list[i], vertex_list[i + 2], invCov)

    graph.solve(mrob.GN)
    print_3d_graph(graph)

    x = graph.get_estimated_state()
    np.save("traj.npy", x)
