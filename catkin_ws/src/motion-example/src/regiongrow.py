import math
import numpy as np
from sklearn.neighbors import KDTree
from scipy import spatial
from open3d import *
import cv2

def normalsestimation(pointcloud,nn_glob,VP=[0,0,0]):
    ViewPoint = np.array(VP)
    normals = np.empty((np.shape(pointcloud)))
    curv    = np.empty((len(pointcloud),1))
    for index in range(len(pointcloud)):
        nn_loc = pointcloud[nn_glob[index]]
        COV = np.cov(nn_loc,rowvar=False)
        eigval, eigvec = np.linalg.eig(COV)
        idx = np.argsort(eigval)
        nor = eigvec[:,idx][:,0]
        if nor.dot((ViewPoint-pointcloud[index,:])) > 0:
            normals[index] = nor
        else:
            normals[index] = - nor
        curv[index] = eigval[idx][0] / np.sum(eigval)
    return normals,curv
#seed_count = 0
#while seed_count < len(current_seeds)

def regiongrowing1(pointcloud,nn_glob,theta_th = 'auto', cur_th = 'auto'):
    normals,curvature = normalsestimation(pointcloud,nn_glob=nn_glob)
    order             = curvature[:,0].argsort().tolist()
    region            = []
    if theta_th == 'auto':
        theta_th          = 5.0 / 180.0 * math.pi # in radians
    if cur_th == 'auto':
        cur_th            = np.percentile(curvature,98)
    while len(order) > 0:
        region_cur = []
        seed_cur   = []
        poi_min    = order[0] #poi_order[0]
        region_cur.append(poi_min)
        seedval = 0

        seed_cur.append(poi_min)
        order.remove(poi_min)
        while seedval < len(seed_cur):
            nn_loc  = nn_glob[seed_cur[seedval]]
            for j in range(len(nn_loc)):
                nn_cur = nn_loc[j]
                if all([nn_cur in order , np.arccos(np.abs(np.dot(normals[seed_cur[seedval]],normals[nn_cur])))<theta_th]):
                    region_cur.append(nn_cur)
                    order.remove(nn_cur)
                    if curvature[nn_cur] < cur_th:
                        seed_cur.append(nn_cur)
            seedval+=1
        region.append(region_cur)
    return region

def get_grasp(xyz):
    cloud = geometry.PointCloud()
    cloud.points = utility.Vector3dVector(xyz)
    # io.write_point_cloud("test.ply", pcd)

    original = np.array(cloud.points)
    cloud = cloud.voxel_down_sample(voxel_size=0.01) # 07)
    points = np.asarray(cloud.points)
    indices = np.where(points[:,2] < 0.6) # 0.67)
    points = points[indices]

    tree    = KDTree(points, leaf_size=2) 
    dist,nn_glob = tree.query(points[:len(points)], k=30) 

    region1  = regiongrowing1(points,nn_glob)

    clusters = []
    for cluster in region1:
        if len(cluster) > 50:
            clusters.append(cluster)

    clusters = sorted(clusters, key=len, reverse=True)

    clouds = []
    for i in range(len(clusters)):
        cloud = points[clusters[i]]

        source = geometry.PointCloud()
        source.points = utility.Vector3dVector(cloud)
        source.paint_uniform_color(np.random.rand(3))
        # io.write_point_cloud("cloud.ply", source)
        clouds.append(source)

    obj = points[clusters[0]]
    centroid = np.mean(obj, axis=0)

    tree = spatial.KDTree(original)
    idx = tree.query(centroid)
    index = idx[1]
    grasp = original[index]

    col = index / 480
    row = index % 480

    frame = geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=grasp)
    clouds.append(frame)
    visualization.draw_geometries(clouds)

    return col, row, grasp

if __name__ == '__main__':
    cloud = io.read_point_cloud("test.ply")

    col, row, grasp, clouds = get_grasp(cloud)

    # img = cv2.imread('cloud.png')
    # cv2.circle(img, (int(col), int(row)), 5, (0, 255, 0), -1)
    # cv2.imshow('img', img)
    # cv2.waitKey()

    print(grasp)




