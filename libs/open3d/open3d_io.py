#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import open3d as o3d

img = o3d.io.read_image("../../data/ad.jpg")
print(img)

pcd = o3d.io.read_point_cloud("../../data/3d/table.pcd")
print(pcd)

# pcd = pcd.voxel_down_sample(voxel_size=0.01)
# print(pcd)
# o3d.io.write_point_cloud("../../data/3d/table.pcd", pcd)

pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

o3d.visualization.draw_geometries([pcd], window_name="点云PCD", width=800, height=600)

create_type = 4

if create_type == 0:
    alpha = 0.1
    print(f"alpha={alpha:.3f}")
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
    mesh.compute_vertex_normals()
    show_info = [mesh, "Triangle Mesh (Alpha Shape)"]

if create_type == 1:
    radii = [0.005, 0.01, 0.02, 0.04]
    bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))
    show_info = [bpa_mesh, "Triangle Mesh (Ball Pivoting)"]

if create_type == 2:
    pcd.paint_uniform_color([1.0, 0.0, 1.0])
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=10))
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=12)
    print(mesh)
    show_info = [mesh, "Triangle Mesh (Poisson)"]

if create_type == 3:
    pcd.paint_uniform_color([1.0, 0.0, 1.0])
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=0.01)
    show_info = [voxel_grid, "Voxel Grid"]

if create_type == 4:
    pont_idx = 5000
    pcd.paint_uniform_color([0.5, 0.5, 0.5])
    pcd_tree = o3d.geometry.KDTreeFlann(pcd)
    print("Paint the 1500th point red.")
    pcd.colors[pont_idx] = [1, 0, 0]
    print("Find its 200 nearest neighbors, paint blue.")
    [k, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[pont_idx], 200)
    np.asarray(pcd.colors)[idx[1:], :] = [0, 0, 1]
    # print("Find its neighbors with distance less than 0.2, paint green.")
    # [k, idx, _] = pcd_tree.search_radius_vector_3d(pcd.points[pont_idx], 0.2)
    # np.asarray(pcd.colors)[idx[1:], :] = [0, 1, 0]
    show_info = [pcd, "KDTreeFlann"]

o3d.visualization.draw_geometries([show_info[0]], window_name=show_info[1], width=800, height=600,
                                  mesh_show_back_face=True)
