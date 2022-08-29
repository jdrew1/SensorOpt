import argparse
import carla
import random
import open3d as o3d
import time
import numpy as np
import math

# Global variables
global client
global world
global bp_lib
global lidar_bp
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
    rand_spawn_point = random.randint(0, len(spawn_points))

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


def place_cylinder_and_car(inputstring = "D:10.0,S:10"):
    distance = float(inputstring.split(',')[0].split(':')[1])
    step = int(inputstring.split(',')[1].split(':')[1])
    global spawn_points
    global vehicle
    global bp_lib
    global rand_spawn_point
    # Add vehicle
    vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020')
    vehicle = world.try_spawn_actor(vehicle_bp, spawn_points[rand_spawn_point])
    # Spawn test cylinder
    box_bp = bp_lib.filter('box02*')[0]
    for theta in range(0, 360, step):
        for height in range(0, 6):
            world.try_spawn_actor(box_bp,
                                  carla.Transform(
                                      spawn_points[rand_spawn_point].transform(
                                          carla.Location(
                                              x= distance * math.cos(theta*math.pi/180),
                                              y= distance * math.sin(theta*math.pi/180),
                                              z= height*0.5-1)),
                                  carla.Rotation(yaw=theta))
                                  )


def find_car_mesh():
    return


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

    point_list.points = o3d.utility.Vector3dVector(points)


def closeEnvironment(stringInput = ""):
    global world
    for actor in world.get_actors().filter('*vehicle*'):
        actor.destroy()
    for actor in world.get_actors().filter('*sensor*'):
        actor.stop()
        actor.destroy()


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
    for i in range(10):
        trainLoop()
        time.sleep(0.1)
    closeEnvironment()
