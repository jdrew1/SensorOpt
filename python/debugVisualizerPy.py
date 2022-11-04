import open3d
import time
import numpy as np


def debugVisualizer(inputstring = ""):
    inputstring = inputstring[1:]
    time.sleep(1.0)
    point_list = open3d.geometry.PointCloud()
    points_to_convert = open3d.utility.Vector3dVector()
    for point in inputstring.split('|'):
        points_to_convert.append([point.split(',')[0], point.split(',')[1], point.split(',')[2]])
    point_list = open3d.geometry.PointCloud(points_to_convert)
    vis = open3d.visualization.VisualizerWithKeyCallback()
    point_list.paint_uniform_color([0.0, 0.0, 0.0])
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
    vis.get_render_option().point_size = 2
    vis.get_render_option().show_coordinate_frame = True
    """Add a small 3D axis on Open3D Visualizer"""
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
            vis.add_geometry(axis)
        vis.update_geometry(point_list)
        vis.poll_events()
        vis.update_renderer()
        time.sleep(0.005)
        frame += 1
    vis.destroy_window()
    return


def debugVisualizerWithColors(inputstring = ""):
    inputstring = inputstring[1:]
    time.sleep(1.0)
    point_list = open3d.geometry.PointCloud()
    points_to_convert = open3d.utility.Vector3dVector()
    for point in inputstring.split('|'):
        points_to_convert.append([point.split(',')[0], point.split(',')[1], point.split(',')[2]])
    point_list = open3d.geometry.PointCloud(points_to_convert)
    vis = open3d.visualization.VisualizerWithKeyCallback()
    point_list.paint_uniform_color([0.0, 0.0, 0.0])
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
    vis.get_render_option().point_size = 2
    vis.get_render_option().show_coordinate_frame = True
    """Add a small 3D axis on Open3D Visualizer"""
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
            vis.add_geometry(axis)
        vis.update_geometry(point_list)
        vis.poll_events()
        vis.update_renderer()
        time.sleep(0.005)
        frame += 1
    vis.destroy_window()
    return
