import argparse
import carla
import random
import open3d
import time
import numpy as np
from matplotlib import cm
import math
import threading

# Global variables
global client
global world
global bp_lib
global lidar_bp
global car_lidar
global lidar
global vehicle
global args
global spectator
global spawn_points
global rand_spawn_point


def setupEnvironment(inputstring = ""):
    # Connect the client and set up bp library and spawn point
    global args
    global client
    client = carla.Client(args.host, args.port)
    global world
    world = client.get_world()
    world.unload_map_layer(carla.MapLayer.All)
    world.load_map_layer(carla.MapLayer.Ground)
    global bp_lib
    bp_lib = world.get_blueprint_library()
    global spawn_points
    spawn_points = world.get_map().get_spawn_points()
    global rand_spawn_point
    rand_spawn_point = random.randint(0, len(spawn_points)-1)

    # Move spectator to view ego vehicle
    global spectator
    spectator = world.get_spectator()
    transform = carla.Transform(spawn_points[rand_spawn_point].transform(carla.Location(x=-4, z=2.5)), carla.Rotation())
    spectator.set_transform(transform)

    # Add traffic and set in motion with Traffic Manager
    if args.traffic:
        for j in range(100):
            vehicle_bp = random.choice(bp_lib.filter('vehicle'))
            npc = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))
        for v in world.get_actors().filter('*vehicle*'):
            v.set_autopilot(True)
    global lidar_bp
    lidar_bp = bp_lib.find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('range', '100.0')
    lidar_bp.set_attribute('noise_stddev', '0.1')
    lidar_bp.set_attribute('upper_fov', '15.0')
    lidar_bp.set_attribute('lower_fov', '-25.0')
    lidar_bp.set_attribute('channels', '64.0')
    lidar_bp.set_attribute('rotation_frequency', '20.0')
    lidar_bp.set_attribute('points_per_second', '500000')


def place_cylinder_and_car(inputstring = "D:10.0"):
    distance = float(inputstring.split(',')[0].split(':')[1])
    global spawn_points
    global vehicle
    global bp_lib
    # Add vehicle
    vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020')
    global spawn_points
    global rand_spawn_point
    vehicle = world.try_spawn_actor(vehicle_bp, spawn_points[rand_spawn_point])
    # Spawn test cylinder
    box_bp = bp_lib.filter('box02*')[0]
    for theta in range(0, 360, 2):
        for height in range(0, 1):
            world.try_spawn_actor(box_bp,
                                  carla.Transform(
                                      spawn_points[rand_spawn_point].transform(
                                          carla.Location(
                                              x= distance * math.cos(theta*math.pi/180),
                                              y= distance * math.sin(theta*math.pi/180),
                                              z= height*0.5-1)),
                                      carla.Rotation(yaw=theta))
                                  )


def find_car_mesh(inputstring = ""):
    distance = 8.5
    # spawn sensor
    global world
    global bp_lib
    global lidar_bp
    global spawn_points
    global rand_spawn_point
    global vehicle
    # place semantic sensor
    car_lidar_bp = bp_lib.find('sensor.lidar.ray_cast_semantic')
    global car_lidar
    car_lidar = world.spawn_actor(car_lidar_bp, carla.Transform(spawn_points[rand_spawn_point].transform(carla.Location(x=distance, z=4))))
    # measure while rotating around car
    total_points = open3d.geometry.PointCloud()
    single_detection = open3d.geometry.PointCloud()

    car_lidar.listen(lambda data: lidar_car_callback(data, single_detection, car_lidar.get_transform(), vehicle.semantic_tags[0]))
    for theta in range(720):
        car_lidar.set_transform(carla.Transform(spawn_points[rand_spawn_point].transform(carla.Location(
                                                    x= distance * math.cos(theta*math.pi/180),
                                                    y= distance * math.sin(theta*math.pi/180),
                                                    z= 4
                                                ))))
        # if points were collected, add them to the output
        if single_detection.has_points():
            if open3d.geometry.PointCloud(total_points).is_empty():
                total_points = single_detection
            else:
                temp_array = np.vstack((np.asarray(single_detection.points), np.asarray(open3d.geometry.PointCloud(total_points).points)))
                total_points = open3d.utility.Vector3dVector(temp_array)

        time.sleep(0.01)
    car_lidar.stop()
    car_lidar.destroy()
    return open3d.geometry.PointCloud(total_points)


def lidar_car_callback(point_cloud, point_list, sensor_transform, vehicle_index):
    points = []
    # filter for points touching car
    for detection in point_cloud:
        if detection.object_tag == vehicle_index:
            points.append(sensor_transform.transform(detection.point).x)
            # for some reason, transforming the y coordinate causes smearing of the point cloud
            points.append(detection.point.y)
            points.append(sensor_transform.transform(detection.point).z)
    # shape point array into open 3d point cloud
    data = np.array(points, float)
    data = data.reshape((int(data.size/3), 3))
    data[:, :1] = (-data[:, :1])
    point_list.points = open3d.utility.Vector3dVector(data)
    point_list.paint_uniform_color([1.0, 1.0, 1.0])


def place_sensors(inputstring = "X:0.0,Y:0.0,Z:0.0"):
    return


def fetch_lidar_measurement():
    return


def clean_for_next_iteration():
    return


# LIDAR callback
def lidar_callback(point_cloud, point_list):
    """reshape to an array of n 4d points [n,4]"""
    data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4')))
    data = np.reshape(data, (int(data.shape[0] / 4), 4))
    """lidar returns x-y-z-d, discard d for x-y-z"""
    points = data[:, :-1]

    """reverse y axis to change from unreal coords to carla coords"""
    points[:, :1] = -points[:, :1]

    point_list.points = open3d.utility.Vector3dVector(points)


def closeEnvironment(stringInput = ""):
    global world
    for actor in world.get_actors().filter('*vehicle*'):
        actor.destroy()
    for actor in world.get_actors().filter('*sensor*'):
        actor.stop()
        actor.destroy()


def debugVisualizer(point_list):
    time.sleep(1.0)

    global car_lidar
    global vehicle
    vis = open3d.visualization.VisualizerWithKeyCallback()

    # point_list.paint_uniform_color([0.0, 0.0, 0.0])

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

    frame = False
    while windowOpen:
        if not frame and not open3d.geometry.PointCloud(point_list).is_empty():
            vis.add_geometry(open3d.geometry.PointCloud(point_list))
            frame = True
        if frame:
            vis.update_geometry(open3d.geometry.PointCloud(point_list))
        vis.poll_events()
        vis.update_renderer()
        time.sleep(0.005)


def parseArguments(inputArgs = None):
    # """Start function"""
    argparser = argparse.ArgumentParser(
        description='CARLA Sensor sync and projection tutorial')
    argparser.add_argument(
        '--host',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-t', '--traffic',
        help='enable traffic during simulation',
        action="store_true")
    argparser.add_argument(
        '-d', '--distance',
        default=20.0,
        type=float,
        help='distance to test cylinder')
    global args
    if inputArgs is None:
        args = argparser.parse_args()
    else:
        args = argparser.parse_args(inputArgs.split())


if __name__ == '__main__':
    parseArguments()
    setupEnvironment("")
    place_cylinder_and_car()
    list_of_points = find_car_mesh()
    debugVisualizer(list_of_points)
    # closeEnvironment()
