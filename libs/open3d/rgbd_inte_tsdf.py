#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Ref: http://www.open3d.org/docs/release/tutorial/pipelines/rgbd_integration.html

import numpy as np
import open3d as o3d


class CameraPose:

    def __init__(self, meta, mat):
        self.metadata = meta
        self.pose = mat

    def __str__(self):
        return 'Metadata : ' + ' '.join(map(str, self.metadata)) + '\n' + \
               "Pose : " + "\n" + np.array_str(self.pose)


def read_trajectory(filename):
    traj = []
    with open(filename, 'r') as f:
        metastr = f.readline()
        while metastr:
            metadata = list(map(int, metastr.split()))
            mat = np.zeros(shape=(4, 4))
            for i in range(4):
                matstr = f.readline()
                mat[i, :] = np.fromstring(matstr, dtype=float, sep=' \t')
            traj.append(CameraPose(metadata, mat))
            metastr = f.readline()
    return traj


use_open3d_data = True

redwood_rgbd = o3d.data.SampleRedwoodRGBDImages()

custom_data_root = "/home/cg/projects/dataset/open3d/RGBD"

if use_open3d_data:
    log_path = redwood_rgbd.odometry_log_path
else:
    log_path = custom_data_root + "/odometry.log"

print(".log path: {}".format(log_path))

camera_poses = read_trajectory(log_path)

volume = o3d.pipelines.integration.ScalableTSDFVolume(
    voxel_length=4.0 / 512.0,
    sdf_trunc=0.04,
    color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)

for i in range(len(camera_poses)):
    print("Integrate {:d}-th image into the volume.".format(i))
    if use_open3d_data:
        color_path = redwood_rgbd.color_paths[i]
        depth_path = redwood_rgbd.depth_paths[i]
    else:
        color_path = custom_data_root + "/color/{:05d}.jpg".format(i)
        depth_path = custom_data_root + "/depth/{:05d}.png".format(i)
    color = o3d.io.read_image(color_path)
    depth = o3d.io.read_image(depth_path)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color, depth, depth_trunc=4.0, convert_rgb_to_intensity=False)
    volume.integrate(
        rgbd,
        o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault),
        np.linalg.inv(camera_poses[i].pose))

print("Extract a triangle mesh from the volume and visualize it.")
# Mesh extraction uses the marching cubes algorithm [LorensenAndCline1987].
mesh = volume.extract_triangle_mesh()
mesh.compute_vertex_normals()
o3d.visualization.draw_geometries([mesh],
                                  front=[0.5297, -0.1873, -0.8272],
                                  lookat=[2.0712, 2.0312, 1.7251],
                                  up=[-0.0558, -0.9809, 0.1864],
                                  zoom=0.47)
