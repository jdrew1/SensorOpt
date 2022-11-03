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
global lidar_measurement
global vehicle
global vehicle_points
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
    lidar_bp.set_attribute('noise_stddev', '0.0')
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
    for theta in range(0, 360, 3):
        for height in range(0, 5):
            world.try_spawn_actor(box_bp,
                                  carla.Transform(
                                      spawn_points[rand_spawn_point].transform(
                                          carla.Location(
                                              x= distance * math.cos(theta*math.pi/180),
                                              y= distance * math.sin(theta*math.pi/180),
                                              z= height*0.65-0.65)),
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
    global vehicle_points
    # place semantic sensor
    car_lidar_bp = bp_lib.find('sensor.lidar.ray_cast_semantic')
    car_lidar_bp.set_attribute('noise_stddev', '0.0')
    global car_lidar
    car_lidar = world.spawn_actor(car_lidar_bp, carla.Transform(spawn_points[rand_spawn_point].transform(carla.Location(x=distance, z=4))))
    # measure while rotating around car
    total_points = open3d.geometry.PointCloud()
    single_detection = open3d.geometry.PointCloud()

    car_lidar.listen(lambda data: lidar_car_callback(data, single_detection, car_lidar.get_transform(), vehicle.semantic_tags[0]))
    for theta in range(0, 720, 2):
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
    # format output point cloud and move to open-3d origin
    vehicle_points = open3d.geometry.PointCloud(total_points)
    vehicle_points.translate([0, 0, 0], False)
    return vehicle_points


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


def place_sensors(inputstring = "0.0,0.0,0.0"):
    global vehicle_points
    global lidar
    global lidar_bp
    global spawn_points
    global rand_spawn_point
    numberOfPoints = inputstring.count('|') + 1
    # place sensors one by one:
    for i in range(numberOfPoints):
        # interpret the points from the input string
        coords = np.array(
                          [float(inputstring.split('|')[i].split(',')[0]),
                           float(inputstring.split('|')[i].split(',')[1]),
                           float(inputstring.split('|')[i].split(',')[2])]
                         )
        coords = coords.reshape((1, 3))
        # find the closest point on car to desired location
        distance_map = vehicle_points.compute_point_cloud_distance(
                       open3d.geometry.PointCloud(open3d.utility.Vector3dVector(coords))
                                                                  )
        closest_point_index = 0
        distance_to_closest_point = 100000.0
        for point in range(len(distance_map)):
            if distance_map[point] < distance_to_closest_point:
                closest_point_index = point
                distance_to_closest_point = distance_map[point]
        # use the normal map to offset the sensor a bit off the car
        vehicle_points.estimate_normals()
        vehicle_points.orient_normals_to_align_with_direction(vehicle_points.points[closest_point_index])
        # spawn the sensor
        lidar = world.spawn_actor(lidar_bp, carla.Transform(carla.Location(
                            x=vehicle_points.points[closest_point_index][0] + 0.1*vehicle_points.normals[closest_point_index][0] + spawn_points[rand_spawn_point].location.x,
                            y=vehicle_points.points[closest_point_index][1] + 0.1*vehicle_points.normals[closest_point_index][1] + spawn_points[rand_spawn_point].location.y,
                            z=vehicle_points.points[closest_point_index][2] + 0.1*vehicle_points.normals[closest_point_index][2] + spawn_points[rand_spawn_point].location.z))
                                      )
        # spawn the occlusion box for the sensor
    return  # list of sensors


def fetch_lidar_measurement(inputstring = ""):
    global spawn_points
    global rand_spawn_point
    global vehicle
    global lidar
    global lidar_measurement
    # start listening to the sensors
    total_points = open3d.geometry.PointCloud()
    single_detection = open3d.geometry.PointCloud()
    lidar.listen(lambda data: lidar_callback(data, single_detection))
    # if points were collected, add them to the output
    j = 0
    while j < 10:
        if single_detection.has_points():
            j = j + 1
            if open3d.geometry.PointCloud(total_points).is_empty():
                total_points = single_detection
            else:
                temp_array = np.vstack((np.asarray(single_detection.points), np.asarray(open3d.geometry.PointCloud(total_points).points)))
                total_points = open3d.utility.Vector3dVector(temp_array)
            time.sleep(0.01)
    lidar.stop()
    lidar.destroy()

    # return the list of points to be converted to cylindrical coordinates and filtered
    lidar_measurement = open3d.geometry.PointCloud(total_points)
    return open3d.geometry.PointCloud(total_points)


def clean_for_next_iteration():
    # stop and delete sensors from vehicle
    # delete vehicle and spawn a new one(?)
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
    point_list.paint_uniform_color([1.0, 1.0, 1.0])


def fetch_vehicle_bounding_box(inputstring = ""):
    global vehicle
    return vehicle.bounding_box.extent


def closeEnvironment(inputstring = ""):
    global world
    for actor in world.get_actors().filter('*vehicle*'):
        actor.destroy()
    for actor in world.get_actors().filter('*sensor*'):
        actor.stop()
        actor.destroy()


def debugVisualizer(inputstring = ""):
    print("started python debugger")
    inputPoints = False
    if inputstring != "":
        inputPoints = True
        inputstring = inputstring[1:]
    time.sleep(1.0)
    global spawn_points
    global rand_spawn_point
    global vehicle_points
    point_list = open3d.geometry.PointCloud()
    global lidar
    global lidar_measurement
    # lidar.listen(lambda data: lidar_callback(data, point_list))
    points_to_convert = open3d.utility.Vector3dVector()
    if inputPoints:
        for point in inputstring.split('|'):
            points_to_convert.append([point.split(',')[0], point.split(',')[1], point.split(',')[2]])
        point_list = open3d.geometry.PointCloud(points_to_convert)
    else:
        point_list = lidar_measurement
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
            # vis.add_geometry(vehicle_points)
        vis.update_geometry(point_list)
        vis.poll_events()
        vis.update_renderer()
        time.sleep(0.005)
        frame += 1


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
    find_car_mesh()
    place_sensors("1.82,0.47,0.65")
    fetch_lidar_measurement()
    debugVisualizer()
    # closeEnvironment()
