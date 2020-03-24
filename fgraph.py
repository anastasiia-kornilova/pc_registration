import mrob
import numpy as np
import glob
import open3d
import matplotlib.pyplot as plt


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
    n1 = graph.add_node_pose_3d(x1)
    # Anchor pose
    graph.add_factor_1pose_3d(np.zeros(6), n1, invCov * 1e6)

    trans1 = find_transformation(pcds[1], pcds[0], np.eye(4))
    x2 = mrob.SE3(trans1 @ mrob.SE3(x1).T()).ln()
    n2 = graph.add_node_pose_3d(x2)
    graph.add_factor_2poses_3d(mrob.SE3(trans1).ln(), n1, n2, invCov)

    trans2 = find_transformation(pcds[2], pcds[1], np.eye(4))
    x3 = mrob.SE3(trans2 @ mrob.SE3(x2).T()).ln()
    n3 = graph.add_node_pose_3d(x3)
    graph.add_factor_2poses_3d(mrob.SE3(trans2).ln(), n2, n3, invCov)

    trans3 = find_transformation(pcds[2], pcds[0], np.eye(4))
    print(trans3)
    print(trans2 @ trans1)
    graph.add_factor_2poses_3d(mrob.SE3(trans3).ln(), n1, n3, invCov * 1e3)

    graph.solve(mrob.GN)
    print_3d_graph(graph)
