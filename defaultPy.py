import carla
import math
import random
import time
import numpy as np
import cv2
import open3d as o3d
from matplotlib import cm

# Connect the client and set up bp library and spawn point
client = carla.Client('localhost', 2000)
world = client.get_world()
bp_lib = world.get_blueprint_library()
spawn_points = world.get_map().get_spawn_points()

# Add vehicle
vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020')
vehicle = world.try_spawn_actor(vehicle_bp, spawn_points[79])

# Move spectator to view ego vehicle
spectator = world.get_spectator()
transform = carla.Transform(vehicle.get_transform().transform(carla.Location(x=-4, z=2.5)), vehicle.get_transform().rotation)
spectator.set_transform(transform)

# Add traffic and set in motion with Traffic Manager
for i in range(100):
    vehicle_bp = random.choice(bp_lib.filter('vehicle'))
    npc = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))
for v in world.get_actors().filter('*vehicle*'):
    v.set_autopilot(True)

# Auxilliary code for colormaps and axes
VIRIDIS = np.array(cm.get_cmap('plasma').colors)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])

COOL_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])
COOL = np.array(cm.get_cmap('winter')(COOL_RANGE))
COOL = COOL[:,:3]


def add_open3d_axis(vis):
    """Add a small 3D axis on Open3D Visualizer"""
    axis = o3d.geometry.LineSet()
    axis.points = o3d.utility.Vector3dVector(np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]]))
    axis.lines = o3d.utility.Vector2iVector(np.array([
        [0, 1],
        [0, 2],
        [0, 3]]))
    axis.colors = o3d.utility.Vector3dVector(np.array([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]]))
    vis.add_geometry(axis)


# LIDAR callback
def lidar_callback(point_cloud, point_list):
    """Prepares a point cloud with intensity
    colors ready to be consumed by Open3D"""
    data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4')))
    data = np.reshape(data, (int(data.shape[0] / 4), 4))

    # Isolate the intensity and compute a color for it
    intensity = data[:, -1]
    intensity_col = 1.0 - np.log(intensity) / np.log(np.exp(-0.004 * 100))
    int_color = np.c_[
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 0]),
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 1]),
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 2])]

    points = data[:, :-1]

    points[:, :1] = -points[:, :1]

    point_list.points = o3d.utility.Vector3dVector(points)
    point_list.colors = o3d.utility.Vector3dVector(int_color)


def open3d_callback(closing):
    closing = True


# Set up LIDAR and RADAR, parameters are to assist visualisation
lidar_bp = bp_lib.find('sensor.lidar.ray_cast')
lidar_bp.set_attribute('range', '100.0')
lidar_bp.set_attribute('noise_stddev', '0.1')
lidar_bp.set_attribute('upper_fov', '15.0')
lidar_bp.set_attribute('lower_fov', '-25.0')
lidar_bp.set_attribute('channels', '64.0')
lidar_bp.set_attribute('rotation_frequency', '20.0')
lidar_bp.set_attribute('points_per_second', '500000')

lidar_init_trans = carla.Transform(carla.Location(z=2))
lidar = world.spawn_actor(lidar_bp, lidar_init_trans, attach_to=vehicle)


# Add auxilliary data structures
point_list = o3d.geometry.PointCloud()


# Start sensors
lidar.listen(lambda data: lidar_callback(data, point_list))


# Open3D visualiser for LIDAR and RADAR
closeRequested = False
vis = o3d.visualization.VisualizerWithKeyCallback()
vis.register_key_callback(ord("Q"), open3d_callback(closeRequested))
vis.create_window(
    window_name='Carla Lidar',
    width=960,
    height=540,
    left=480,
    top=270)
vis.get_render_option().background_color = [0.05, 0.05, 0.05]
vis.get_render_option().point_size = 1
vis.get_render_option().show_coordinate_frame = True
add_open3d_axis(vis)

# Update geometry and camera in game loop
frame = 0
while True:
    if frame == 2:
        vis.add_geometry(point_list)
    vis.update_geometry(point_list)

    vis.poll_events()
    vis.update_renderer()
    # # This can fix Open3D jittering issues:
    time.sleep(0.005)
    frame += 1


    # Break if user presses 'q'
    if closeRequested:
        break

lidar.stop()
lidar.destroy()
vis.destroy_window()

for actor in world.get_actors().filter('*vehicle*'):
    actor.destroy()
for actor in world.get_actors().filter('*sensor*'):
    actor.destroy()
