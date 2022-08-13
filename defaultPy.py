import argparse
import carla
import random
import open3d as o3d
import numpy as np

# Global variables
global client
global world
global bp_lib
global lidar_bp
global lidar
global vehicle
global args


def setupEnvironment(inputstring):
    # Connect the client and set up bp library and spawn point
    global client
    client = carla.Client(args.host, args.port)
    global world
    world = client.get_world()
    global bp_lib
    bp_lib = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()

    # Add vehicle
    vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020')
    global vehicle
    vehicle = world.try_spawn_actor(vehicle_bp, spawn_points[79])

    # Move spectator to view ego vehicle
    spectator = world.get_spectator()
    transform = carla.Transform(vehicle.get_transform().transform(carla.Location(x=-4, z=2.5)), vehicle.get_transform().rotation)
    spectator.set_transform(transform)

    # Add traffic and set in motion with Traffic Manager
    if args.traffic:
        for i in range(100):
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


def trainLoop(inputargs):
    if inputargs is tuple:
        args = inputargs[0]
        inputstring = inputargs[1]
    print(inputargs)
    print(args)
    print(inputstring)

    lidar_init_trans = carla.Transform(carla.Location(z=2))
    global lidar
    lidar = world.spawn_actor(lidar_bp, lidar_init_trans, attach_to=vehicle)
    # Add auxilliary data structures
    point_list = o3d.geometry.PointCloud()
    # Start sensors
    lidar.listen(lambda data: lidar_callback(data, point_list))
    return point_list


# LIDAR callback
def lidar_callback(point_cloud, point_list):
    """Prepares a point cloud with intensity
    colors ready to be consumed by Open3D"""
    data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4')))
    data = np.reshape(data, (int(data.shape[0] / 4), 4))
    points = data[:, :-1]
    points[:, :1] = -points[:, :1]
    point_list.points = o3d.utility.Vector3dVector(points)


def closeEnvironment():
    try:
        lidar.stop()
        lidar.destroy()
    except:
        print("no lidar to destroy")

    global world
    for actor in world.get_actors().filter('*vehicle*'):
        actor.destroy()
    for actor in world.get_actors().filter('*sensor*'):
        actor.destroy()


def parseArguments(inputArgs = None):
    # """Start function"""
    argparser = argparse.ArgumentParser(
        description='CARLA Sensor sync and projection tutorial')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-t', '--traffic',
        metavar='T',
        default=True,
        type=bool,
        help='choose whether to spawn traffic in sim')
    global args
    if inputArgs is None:
        args = argparser.parse_args()
    else:
        args = argparser.parse_args(inputArgs.split())


if __name__ == '__main__':
    setupEnvironment(parseArguments())
    trainLoop(parseArguments())
    closeEnvironment()