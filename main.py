import numpy as np
from numpy.linalg import svd
import open3d as o3d
import datetime


def icp_core(points1, points2):
    """
    solve transformation from points1 to points2, points of the same index are well matched
    :param points1: numpy array of points1, size = nx3, n is num of point
    :param points2: numpy array of points2, size = nx3, n is num of point
    :return: transformation matrix T, size = 4x4
    """
    assert points1.shape == points2.shape, 'point could size not match'

    T = np.zeros(shape=(4, 4))
    T[0:3, 0:3] = np.eye(3)
    T[3, 3] = 1

    # Todo: step1: calculate centroid
    centroid1 = np.mean(points1, axis=0)
    centroid2 = np.mean(points2, axis=0)

    # Todo: step2: de-centroid of points1 and points2
    points1_centered = points1 - centroid1
    points2_centered = points2 - centroid2

    # Todo: step3: compute H, which is sum of p1i'*p2i'^T
    H = np.dot(points1_centered.T, points2_centered)

    # Todo: step4: SVD of H (can use 3rd-part lib), solve R and t
    U, S, Vt = svd(H)
    R = np.dot(Vt.T, U.T)
    t = centroid2.T - np.dot(R, centroid1.T)

    # Todo: step5, combine R and t into transformation matrix T
    T[:3, :3] = R
    T[:3, 3] = t
    T[3, 3] = 1

    return T


def svd_based_icp_matched(points1, points2):

    # icp_core should be finished first
    T = icp_core(points1, points2)
    print('------------transformation matrix------------')
    print(T)

    # Todo: calculate transformed point cloud 1 based on T solved above, and name it pcd1_transformed (same format as point1)
    points1_transformed = np.dot(points1, T[:3, :3].T) + T[:3, 3] # comment this line, this only indicates the size of points_2_nearest should be the same as points1


    # visualization
    mean_distance = mean_dist(points1_transformed, points2)
    print('mean_error= ' + str(mean_distance))
    axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[0, 0, 0])
    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(points1)
    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(points2)
    pcd1_tran = o3d.geometry.PointCloud()
    pcd1_tran.points = o3d.utility.Vector3dVector(points1_transformed)
    pcd1.paint_uniform_color([1, 0, 0]) #point cloud 1 : red
    pcd2.paint_uniform_color([0, 1, 0]) #point cloud 2 : green
    pcd1_tran.paint_uniform_color([0, 0, 1])
    o3d.visualization.draw_geometries([pcd1, pcd2, pcd1_tran, axis_pcd])


def svd_based_icp_unmatched(points1, points2, n_iter, threshold):
    point_size = points1.shape[0]
    points_1 = points1.copy()
    points_2 = points2.copy()
    points_2_nearest = np.empty((point_size,3))
    T_accumulated = np.eye(4)

    axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[0, 0, 0])
    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(points_2)
    pcd2.paint_uniform_color([0, 1, 0]) #point cloud 2 : green
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(axis_pcd)
    vis.add_geometry(pcd2)

    start_time = datetime.datetime.now()

    for i in range(n_iter):

        # Todo: for all point in points_1, find nearest point in points_2, and generate points_2_nearest
        array_intermediate = np.empty((point_size,3))
        for j in range(point_size):
            query_point = points_1[j]
            distance = np.sqrt(np.sum((points2 - query_point)**2, axis = 1))
            nearest_point_index = np.argmin(distance)
            array_intermediate[j] = points2[nearest_point_index]
        
        points_2_nearest = array_intermediate

        # solve icp
        T = icp_core(points_1, points_2_nearest)

        # Todo: update accumulated T
        T_accumulated = np.dot(T_accumulated,T)

        print('-----------------------------------------')
        print('iteration = ' + str(i+1))
        print('T = ')
        print(T)
        print('accumulated T = ')
        print(T_accumulated)

        # Todo: update points_1
        points_1 = np.dot(points_1, T[:3, :3].T) + T[:3, 3]

        mean_distance = mean_dist(points_1, points2)
        print('mean_error= ' + str(mean_distance))

        # visualization
        pcd1_transed = o3d.geometry.PointCloud()
        pcd1_transed.points = o3d.utility.Vector3dVector(points_1)
        pcd1_transed.paint_uniform_color([0, 0, 1]) # transformed point cloud 1: blue
        vis.add_geometry(pcd1_transed)
        vis.poll_events()
        vis.update_renderer()
        # fig_name = "image_" + str(i) + ".png"
        # vis.capture_screen_image(fig_name)
        vis.remove_geometry(pcd1_transed)

        if mean_distance < 0.00001 or mean_distance < threshold:
            print('fully converged!')
            break

    end_time = datetime.datetime.now()
    time_difference = (end_time - start_time).total_seconds()
    print('time cost: ' + str(time_difference) + 's')
    vis.destroy_window()
    o3d.visualization.draw_geometries([axis_pcd, pcd2, pcd1_transed])


def mean_dist(points1, points2):
    dis_array = []
    for i in range(points1.shape[0]):
        dif = points1[i] - points2[i]
        dis = np.linalg.norm(dif)
        dis_array.append(dis)
    return np.mean(np.array(dis_array))


def main():
    pcd1 = o3d.io.read_point_cloud('bunny1.ply')
    pcd2 = o3d.io.read_point_cloud('bunny2.ply')
    points1 = np.array(pcd1.points)
    points2 = np.array(pcd2.points)

    # task 1:
    # svd_based_icp_matched(points1, points2)
    # task 2:
    svd_based_icp_unmatched(points1, points2, n_iter=30, threshold=0.1)

if __name__ == '__main__':
    main()
