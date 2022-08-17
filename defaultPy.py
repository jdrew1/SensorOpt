import argparse
import carla
import random
import open3d as o3d
import time
import numpy as np

# Global variables
global client
global world
global bp_lib
global lidar_bp
global lidar
global vehicle
global args
global spectator


def setupEnvironment(inputstring = ""):
    # Connect the client and set up bp library and spawn point
    global args
    global client
    client = carla.Client(args.host, args.port)
    global world
    world = client.get_world()
    world.unload_map_layer(carla.MapLayer.Buildings)
    world.unload_map_layer(carla.MapLayer.Foliage)
    world.unload_map_layer(carla.MapLayer.Particles)
    global bp_lib
    bp_lib = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()

    # Add vehicle
    vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020')
    global vehicle
    vehicle = world.try_spawn_actor(vehicle_bp, spawn_points[79])

    # Move spectator to view ego vehicle
    global spectator
    spectator = world.get_spectator()
    transform = carla.Transform(vehicle.get_transform().transform(carla.Location(x=-4, z=2.5)), vehicle.get_transform().rotation)
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


def trainLoop(inputstring = "X:0.0,Y:0.0,Z:0.0"):
    coords = [float(inputstring.split(',')[0].split(':')[1]),
              float(inputstring.split(',')[1].split(':')[1]),
              float(inputstring.split(',')[2].split(':')[1])]
    lidar_init_trans = carla.Transform(carla.Location(x=coords[0], y=coords[1], z=coords[2]))
    global lidar
    lidar = world.spawn_actor(lidar_bp, lidar_init_trans, attach_to=vehicle)
    point_list = o3d.geometry.PointCloud()
    lidar.listen(lambda data: lidar_callback(data, point_list))
    time.sleep(0.5)
    # Add auxilliary data structures
    # Start sensors
    lidar.stop()
    lidar.destroy()
    return point_list


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
