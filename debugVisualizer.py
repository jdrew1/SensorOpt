import open3d
import time
import numpy as np
import math
from matplotlib import cm
import CarlaInterface


def convert_to_cartesian(cylindrical_points):
    def conv_func(cyl_points=np.array):
        return np.array([cyl_points[0]*math.cos(cyl_points[1]),
                         cyl_points[0]*math.sin(cyl_points[1]),
                         cyl_points[2]])
    cartesian_points = np.apply_along_axis(conv_func, -1, cylindrical_points)
    return cartesian_points


def debugVisualizer(input_points, cylindrical=False, colors=False, color_list=np.zeros(shape=(0, 3)), downsample=False):
    draw_axis = True
    color_map = np.array(cm.get_cmap('viridis').colors)
    color_range = np.linspace(0.0, 1.0, color_map.shape[0])


    point_list = open3d.geometry.PointCloud()
    if type(input_points) is np.ndarray:
        point_list = open3d.geometry.PointCloud(open3d.utility.Vector3dVector(input_points))
    if cylindrical:
        point_list = open3d.geometry.PointCloud(open3d.utility.Vector3dVector(convert_to_cartesian(input_points)))
    if type(input_points) is open3d.geometry.PointCloud:
        point_list = input_points
    if downsample:
        point_list = point_list.voxel_down_sample(voxel_size=0.1)
        input_points = np.asarray(point_list.points)

    vis = open3d.visualization.VisualizerWithKeyCallback()
    if colors:
        if color_list.shape != input_points.shape:
            color_list = np.zeros(shape=input_points.shape)
            for point in range(input_points.shape[0]):
                intensity = input_points[point, 2] / 3.0
                color_list[point] = np.array([np.interp(intensity, color_range, color_map[:, 0]),
                                              np.interp(intensity, color_range, color_map[:, 1]),
                                              np.interp(intensity, color_range, color_map[:, 2])])
        point_list.colors = open3d.utility.Vector3dVector(color_list)
    #else:
        #point_list.paint_uniform_color([0.0, 0.0, 0.0])
    windowOpen = True

    def keyCallback(vis, action, mods):
        nonlocal windowOpen
        windowOpen = False
        vis.close()
        return True
    vis.register_key_action_callback(32, keyCallback)
    vis.create_window(
        window_name='Carla Lidar',
        width=960,
        height=540,
        left=480,
        top=270)
    vis.get_render_option().background_color = [1.0, 1.0, 1.0]
    vis.get_render_option().point_size = 5
    vis.get_render_option().show_coordinate_frame = True
    """Add a small 3D axis on Open3D Visualizer"""
    if draw_axis:
        axis = open3d.geometry.LineSet()
        axis.points = open3d.utility.Vector3dVector(np.array([
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]]))
        axis.lines = open3d.utility.Vector2iVector(np.array([
            [0, 1],
            [0, 2],
            [0, 3]]))
        axis.colors = open3d.utility.Vector3dVector(np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]]))
    frame = 0
    while windowOpen:
        if frame == 2:
            vis.add_geometry(point_list)
            if draw_axis:
                vis.add_geometry(axis)
        vis.update_geometry(point_list)
        vis.poll_events()
        vis.update_renderer()
        time.sleep(0.005)
        frame += 1
    vis.destroy_window()
    return
