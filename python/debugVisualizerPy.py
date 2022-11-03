import open3d
import time

# Global variables
global client
global world
global bp_lib
global lidar_bp
global car_lidar
global lidar
global lidar_measurement
global vehicle
global vehicle_points
global args
global spectator
global spawn_points
global rand_spawn_point


def debugVisualizer(inputstring = ""):
    inputPoints = False
    if inputstring != "":
        inputPoints = True
        inputstring = inputstring[1:]
    time.sleep(1.0)
    point_list = open3d.geometry.PointCloud()
    points_to_convert = open3d.utility.Vector3dVector()
    if inputPoints:
        for point in inputstring.split('|'):
            points_to_convert.append([point.split(',')[0], point.split(',')[1], point.split(',')[2]])
        point_list = open3d.geometry.PointCloud(points_to_convert)
    else:
        a = 0
        # point_list = lidar_measurement
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
    vis.get_render_option().point_size = 1
    vis.get_render_option().show_coordinate_frame = True
    """Add a small 3D axis on Open3D Visualizer"""
    """axis = open3d.geometry.LineSet()
    axis.points = open3d.utility.Vector3dVector(np.array([
        [0.0 + lidar.get_transform().location.x - spawn_points[rand_spawn_point].location.x, 0.0 + lidar.get_transform().location.y - spawn_points[rand_spawn_point].location.y, 0.0 + lidar.get_transform().location.z - spawn_points[rand_spawn_point].location.z],
        [1.0 + lidar.get_transform().location.x - spawn_points[rand_spawn_point].location.x, 0.0 + lidar.get_transform().location.y - spawn_points[rand_spawn_point].location.y, 0.0 + lidar.get_transform().location.z - spawn_points[rand_spawn_point].location.z],
        [0.0 + lidar.get_transform().location.x - spawn_points[rand_spawn_point].location.x, 1.0 + lidar.get_transform().location.y - spawn_points[rand_spawn_point].location.y, 0.0 + lidar.get_transform().location.z - spawn_points[rand_spawn_point].location.z],
        [0.0 + lidar.get_transform().location.x - spawn_points[rand_spawn_point].location.x, 0.0 + lidar.get_transform().location.y - spawn_points[rand_spawn_point].location.y, 1.0 + lidar.get_transform().location.z - spawn_points[rand_spawn_point].location.z]]))
    axis.lines = open3d.utility.Vector2iVector(np.array([
        [0, 1],
        [0, 2],
        [0, 3]]))
    axis.colors = open3d.utility.Vector3dVector(np.array([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]]))
    vis.add_geometry(axis)"""
    frame = 0
    while windowOpen:
        if frame == 2:
            vis.add_geometry(point_list)
        vis.update_geometry(point_list)
        vis.poll_events()
        vis.update_renderer()
        time.sleep(0.005)
        frame += 1
