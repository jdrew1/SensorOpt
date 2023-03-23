import carla
import open3d
import time
import numpy as np
import math
import os
import random
from matplotlib import cm

# Global variables
import debugVisualizer

global client
global world
global bp_lib
global lidar_bp
global car_lidar
global lidar
global vehicle
global test_rectangle_flag
global vehicle_bp
global all_vehicle_bp
global vehicle_points
global spectator
global carlaClientApp
global box_semantic_tag
global place_box
global box_points


"""main methods, in order used during training step"""


def setup_carla_environment(no_render=False, host_ip='127.0.0.1', host_port=2000, macos=False):
    global place_box
    place_box = True
    global carlaClientApp
    if macos:
        os.system("open /Users/jordan/carla/Dist/CARLA_Shipping_9607eca-dirty/MacNoEditor/CarlaUE4-Mac-Shipping.app --args -quality-level=Low")
        time.sleep(4)
    # Connect the client and set up bp library and spawn point
    global client
    client = carla.Client(host_ip, host_port)
    global world
    world = client.get_world()
    world.unload_map_layer(carla.MapLayer.All)
    world.load_map_layer(carla.MapLayer.Ground)
    settings = world.get_settings()
    if no_render:
        settings.no_rendering_mode = True
    world.apply_settings(settings)
    global bp_lib
    bp_lib = world.get_blueprint_library()
    global spectator
    spectator = world.get_spectator()
    transform = carla.Transform(carla.Location(x=-4, z=2.5), carla.Rotation())
    spectator.set_transform(transform)
    global lidar_bp
    lidar_bp = bp_lib.find('sensor.lidar.ray_cast_semantic')
    lidar_bp.set_attribute('points_per_second', '4000000')
    lidar_bp.set_attribute('rotation_frequency', '60.0')
    lidar_bp.set_attribute('range', '15.0')
    lidar_bp.set_attribute('channels', '128')
    lidar_bp.set_attribute('upper_fov', '20')
    world.tick()
    return client


def place_cylinder_and_car(distance=10.0):
    global vehicle
    global bp_lib
    global vehicle_bp
    # Add vehicle
    vehicle_bp = bp_lib.find('vehicle.audi.etron')
    vehicle = world.try_spawn_actor(vehicle_bp, carla.Transform(carla.Location(x=0, y=0, z=0.2)))
    # Spawn test cylinder
    box_bp = bp_lib.filter('box02*')[0]
    for theta in range(0, 360, 3):
        for height in np.arange(0, 4, 0.6):
            world.try_spawn_actor(box_bp,carla.Transform(carla.Location(x=distance * math.cos(theta*math.pi/180),
                                                                        y=distance * math.sin(theta*math.pi/180),
                                                                        z=height*0.65),
                                                         carla.Rotation(yaw=theta)))


def findCarMesh(numOfPoints = 6000, setNumber = False, downsample=False):
    distance = 8.5
    # spawn sensor
    global world
    global bp_lib
    global lidar_bp
    global vehicle
    global vehicle_points
    global test_rectangle_flag
    test_rectangle_flag = False
    # place semantic sensor
    car_lidar_bp = bp_lib.find('sensor.lidar.ray_cast_semantic')
    car_lidar_bp.set_attribute('points_per_second', '4000000')
    car_lidar_bp.set_attribute('rotation_frequency', '60.0')
    car_lidar_bp.set_attribute('channels', '64')
    global car_lidar
    total_points = open3d.geometry.PointCloud()
    car_lidar = world.spawn_actor(car_lidar_bp, carla.Transform(carla.Location(x=distance, z=4)))
    # measure while rotating around car
    theta = 0
    temp_array = np.zeros((0, 3))
    collect_initial_points_flag = True
    while True:
        theta += 75
        car_lidar.set_transform(carla.Transform(carla.Location(x=distance * math.cos(theta*math.pi/180),
                                                               y=distance * math.sin(theta*math.pi/180),
                                                               z=4)))
        single_detection = open3d.geometry.PointCloud()
        car_lidar.listen(lambda data: lidar_car_callback(data, single_detection, car_lidar.get_transform(), vehicle.semantic_tags[0]))
        time.sleep(0.1)
        # if points were collected, add them to the output
        if single_detection.has_points():
            if collect_initial_points_flag:
                total_points = single_detection
                collect_initial_points_flag = False
            else:
                temp_array = np.vstack((np.asarray(single_detection.points), np.asarray(open3d.geometry.PointCloud(total_points).points)))
                total_points = open3d.utility.Vector3dVector(temp_array)
        if (setNumber and temp_array.shape[0] > numOfPoints) or (not setNumber and theta > 720):
            break
        car_lidar.stop()
    car_lidar.destroy()
    # format output point cloud and move to open-3d origin
    vehicle_points = open3d.geometry.PointCloud(total_points).translate([0.0, 0.0, -4.0])
    if downsample:
        vehicle_points = vehicle_points.voxel_down_sample(voxel_size=0.05)
    temp_array = np.asarray(vehicle_points.points)
    if setNumber:
        if temp_array.shape[0] >= numOfPoints:
            temp_array = temp_array[:numOfPoints]
    # debugVisualizer.debugVisualizer(temp_array)
    return temp_array


def testBoxMesh(numOfPoints=6000, setNumber=False, downsample=False):
    # destroy the vehicle and create the box:
    global place_box
    global box_points
    global world
    global test_rectangle_flag
    test_rectangle_flag = True
    if place_box:
        place_box = False
        global vehicle
        vehicle.destroy()
        global bp_lib
        for x in np.arange(-3,3.1,0.5):
            for y in np.arange(-1.5,1.6,0.5):
                for z in np.arange(0,1.1,0.5):
                    box_bp = bp_lib.filter('box02*')[0]
                    world.try_spawn_actor(box_bp,
                                          carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation()))
    distance = 8.5
    # spawn sensor
    global vehicle_points
    # place semantic sensor
    car_lidar_bp = bp_lib.find('sensor.lidar.ray_cast_semantic')
    car_lidar_bp.set_attribute('points_per_second', '1000000')
    car_lidar_bp.set_attribute('rotation_frequency', '60.0')
    car_lidar_bp.set_attribute('channels', '64')
    global car_lidar
    total_points = open3d.geometry.PointCloud()
    car_lidar = world.spawn_actor(car_lidar_bp, carla.Transform(carla.Location(x=distance, z=4)))
    # measure while rotating around car
    theta = 0
    temp_array = np.zeros((0, 3))
    collect_initial_points_flag = True
    while True:
        theta += 105
        car_lidar.set_transform(carla.Transform(carla.Location(x=distance * math.cos(theta*math.pi/180),
                                                               y=distance * math.sin(theta*math.pi/180),
                                                               z=4)))
        single_detection = open3d.geometry.PointCloud()
        car_lidar.listen(lambda data: lidar_car_callback(data, single_detection, car_lidar.get_transform(), 20))
        time.sleep(0.04)
        # if points were collected, add them to the output
        if single_detection.has_points():
            if collect_initial_points_flag:
                total_points = single_detection
                collect_initial_points_flag = False
            else:
                temp_array = np.vstack((np.asarray(single_detection.points), np.asarray(open3d.geometry.PointCloud(total_points).points)))
                filter_array = []
                for point in temp_array:
                    if -5 < point[0] < 5 and -5 < point[1] < 5 and -2 < point[2] < 8.0:
                        filter_array.append(True)
                    else:
                        filter_array.append(False)
                temp_array = temp_array[filter_array]
                total_points = open3d.utility.Vector3dVector(temp_array)
        if (setNumber and temp_array.shape[0] > numOfPoints) or (not setNumber and theta > 720):
            break
        car_lidar.stop()
    car_lidar.destroy()
    # format output point cloud and move to open-3d origin
    vehicle_points = open3d.geometry.PointCloud(total_points).translate([0.0, 0.0, -4.0])
    if downsample:
        vehicle_points = vehicle_points.voxel_down_sample(voxel_size=0.05)
    # bbox = open3d.geometry.AxisAlignedBoundingBox(min_bound=(-3, -3, 0), max_bound=(3, 3, 3))
    temp_array = np.asarray(vehicle_points.points)
    if setNumber:
        if temp_array.shape[0] >= numOfPoints:
            temp_array = temp_array[:numOfPoints]
    # debugVisualizer.debugVisualizer(temp_array)
    return temp_array


def lidar_car_callback(point_cloud, point_list, sensor_transform, vehicle_index):
    points = []
    # filter for points touching car
    for detection in point_cloud:
        if detection.object_tag == vehicle_index:
            # for some reason, transforming the y coordinate causes smearing of the point cloud
            points.append([sensor_transform.transform(detection.point).x,
                           detection.point.y,
                           sensor_transform.transform(detection.point).z])
    # shape point array into open 3d point cloud
    data = np.array(points, float)
    data = data.reshape((int(data.size/3), 3))
    point_list.points = open3d.utility.Vector3dVector(data)


def correct_input_points_to_point_on_vehicle(input_sensor_coords):
    global vehicle_points
    output = input_sensor_coords.reshape((-1, 3))
    # find the closest point on car to desired location
    for i in range(output.shape[0]):
        distance_map = vehicle_points.compute_point_cloud_distance(
                       open3d.geometry.PointCloud(open3d.utility.Vector3dVector(output[i].reshape((1, 3)))))
        closest_point_index = np.argmin(distance_map)
        # use the normal map to offset the sensor a bit off the car
        vehicle_points.estimate_normals()
        vehicle_points.orient_normals_to_align_with_direction(vehicle_points.points[closest_point_index])
        output[i] = np.copy(np.array([[vehicle_points.points[closest_point_index][0] + 0.2*vehicle_points.normals[closest_point_index][0],
                                       vehicle_points.points[closest_point_index][1] + 0.2*vehicle_points.normals[closest_point_index][1],
                                       vehicle_points.points[closest_point_index][2] + 0.2*vehicle_points.normals[closest_point_index][2]]]))
    return output


def placeSensor(coords=np.array, place_sensor_outside_of_vehicle=True):
    global vehicle_points
    global lidar
    global lidar_bp
    global world
    if coords is None:
        coords = np.array([0.0, 0.0, 0.0])
    internal_coords = np.copy(coords.reshape((1, 3)))
    if place_sensor_outside_of_vehicle:
        bounding_box = vehicle.bounding_box.extent
        if abs(internal_coords[0, 0]) > bounding_box.x:
            internal_coords[0, 0] = bounding_box.x if internal_coords[0, 0] > 0 else -bounding_box.x
        if abs(internal_coords[0, 1]) > bounding_box.y:
            internal_coords[0, 1] = bounding_box.y if internal_coords[0, 1] > 0 else -bounding_box.y
        if abs(internal_coords[0, 2]) > bounding_box.z:
            internal_coords[0, 2] = bounding_box.z if internal_coords[0, 2] > 0 else -bounding_box.z
            internal_coords[0, 2] = 0.01 if internal_coords[0, 2] < 0.01 else internal_coords[0, 2]
    # spawn the sensor
    lidar = world.try_spawn_actor(lidar_bp, carla.Transform(carla.Location(x=float(internal_coords[0, 0]),
                                                                           y=float(internal_coords[0, 1]),
                                                                           z=float(internal_coords[0, 2]))))
    return np.asarray([lidar.get_transform().location.x, lidar.get_transform().location.y, lidar.get_transform().location.z])


def lidar_callback(point_cloud, point_list, vehicle_index):
    points = []
    # filter for points touching car
    for detection in point_cloud:
        if detection.object_tag == vehicle_index:
            # for some reason, transforming the y coordinate causes smearing of the point cloud
            points.append([detection.point.x,
                           detection.point.y,
                           detection.point.z])
    # shape point array into open 3d point cloud
    data = np.array(points, float)
    data = data.reshape((int(data.size/3), 3))
    point_list.points = open3d.utility.Vector3dVector(data)


def fetchLiDARMeasurement():
    global vehicle
    global lidar
    global world
    # start listening to the sensors
    total_points = np.zeros(shape=(0, 3))
    single_detection = open3d.geometry.PointCloud()
    lidar.listen(lambda data: lidar_callback(data, single_detection, 20))
    # if points were collected, add them to the output
    j = 0
    start_time = time.thread_time()
    while j < 40:
        if single_detection.has_points():
            j = j + 1
            if total_points.shape == (0, 3):
                total_points = np.asarray(single_detection.points)
            else:
                total_points = np.vstack((np.asarray(single_detection.points), total_points))
            time.sleep(0.02)
        if time.thread_time() - start_time > 10:
            break
    lidar.stop()
    total_points = np.asarray(open3d.geometry.PointCloud(open3d.utility.Vector3dVector(total_points)).translate((lidar.get_transform().location.x, lidar.get_transform().location.y, lidar.get_transform().location.z)).points)
    lidar.destroy()
    # debugVisualizer.debugVisualizer(total_points)
    return total_points


def calculate_test_cylinder_surface(captured_points, cylinder_shape=(3600, 40), down_sample=True):
    if captured_points.shape[0] == 0:
        return np.zeros(shape=cylinder_shape)
    cylinder_face = np.zeros(shape=cylinder_shape)
    if down_sample:
        captured_points = np.asarray(open3d.geometry.PointCloud(open3d.utility.Vector3dVector(captured_points)).voxel_down_sample(voxel_size=0.05).points)

    def calc_func(_p, tlo_surface):
        # do conversion to cylindrical on the fly to prevent looping through the array twice
        if math.sqrt(_p[0] ** 2 + _p[1] ** 2) > 9.0 and 0 <= _p[2] < 4:
            tlo_surface[int((tlo_surface.shape[0]) * (
                math.atan2(_p[1], _p[0]) if _p[1] > 0 else math.atan2(_p[1], _p[0]) + 2 * math.pi) / (2 * math.pi)),
                        int((tlo_surface.shape[1]) * (_p[2] / 4.0))] = 1

    np.apply_along_axis(calc_func, -1, captured_points, cylinder_face)
    return cylinder_face


def cartesian_to_cylindrical_coords(cartesian_points):
    def conv_func(input_points=np.array):
        return np.array([math.sqrt(input_points[0]**2 + input_points[1]**2),
                         math.atan2(input_points[1], input_points[0]) if input_points[1] > 0 else math.atan2(input_points[1], input_points[0]) + 2*math.pi,
                         input_points[2]])
    cylindrical_points = np.apply_along_axis(conv_func, -1, cartesian_points)
    return cylindrical_points


def calculate_sensor_occlusion(input_points, coordinates, index):
    for i in range(coordinates.shape[0]):
        if index == i:
            continue
        distance = math.sqrt((coordinates[i, 0] - coordinates[index, 0])**2 + (coordinates[i, 1] - coordinates[index, 1])**2)
        if distance <= 0.1:
            return np.zeros(shape=input_points.shape)
        top = (coordinates[index, 2] + 10/distance*(coordinates[i, 2] - coordinates[index, 2] + 0.2))
        bottom = (coordinates[index, 2] + 10/distance*(coordinates[i, 2] - coordinates[index, 2] - 0.2))
        left = (math.atan2(coordinates[i, 1] - coordinates[index, 1], coordinates[i, 0] - coordinates[index, 0]) + math.sin(0.4/distance))
        right = (math.atan2(coordinates[i, 1] - coordinates[index, 1], coordinates[i, 0] - coordinates[index, 0]) - math.sin(0.4/distance))
        left = left + 2*math.pi if left < 0 else left
        right = right + 2*math.pi if right < 0 else right
        if top > 4.0:
            top = 4.0
        if bottom < 0:
            bottom = 0
        difference = np.copy(input_points)
        if left > right:
            input_points[int((input_points.shape[0])*right/(2*math.pi)):int((input_points-1).shape[0]*left/(2*math.pi)),
                         int((input_points.shape[1]-1)*bottom/4.0):int((input_points.shape[1]-1)*top/4.0)] = 0
        else:
            input_points[0:int((input_points.shape[0]) * left / (2 * math.pi)),
                         int((input_points.shape[1]) * bottom / 4.0):int((input_points.shape[1] - 1) * top / 4.0)] = 0
            input_points[int(input_points.shape[0]*right/(2*math.pi)):input_points.shape[0],
                         int((input_points.shape[1])*bottom/4.0):int((input_points.shape[1]-1)*top/4.0)] = 0
    return input_points


def calculate_tlo_with_occlusion(coords, cylinder_shape=(360, 40), downsample=True, front_and_back_scaling=True):
    global bp_lib
    global vehicle
    global vehicle_bp
    global world
    global lidar
    if coords is None:
        coords = np.asarray([[1, 1, 2], [-1, 1, 2], [1, -1, 2], [-1, -1, 2]], dtype=float)
    np_coords = np.asarray(coords)
    np_coords = np_coords.reshape((-1, 3))
    total_surface = np.zeros(shape=cylinder_shape)
    for index in range(np_coords.shape[0]):
        occluded_surface = np.zeros(shape=cylinder_shape)
        placeSensor(np_coords[index], place_sensor_outside_of_vehicle=False)
        measure_points = fetchLiDARMeasurement()
        if downsample:
            measure_points = np.asarray(
                open3d.geometry.PointCloud(open3d.utility.Vector3dVector(measure_points)).voxel_down_sample(
                    voxel_size=0.005).points)
        # debugVisualizer.debugVisualizer(measure_points)
        if measure_points.shape == (0, 3):
            continue

        def calc_func(_p, tlo_surface):
            # do conversion to cylindrical on the fly to prevent looping through the array twice
            if math.sqrt(_p[0] ** 2 + _p[1] ** 2) > 9.0 and 0 <= _p[2] < 4:
                tlo_surface[int((tlo_surface.shape[0] - 1) *
                                (math.atan2(_p[1], _p[0]) if _p[1] >= 0 else math.atan2(_p[1], _p[0]) + 2 * math.pi)
                                / (2 * math.pi)),
                            min(tlo_surface.shape[1]-1, int((tlo_surface.shape[1] - 1) * (_p[2] / 3.0)))] = 1

        np.apply_along_axis(calc_func, -1, measure_points, occluded_surface)
        if front_and_back_scaling:
            front_angle = 40
            back_angle = 30
            occluded_surface[int(occluded_surface.shape[0]*(90-front_angle)/360):int(occluded_surface.shape[0]*(90+front_angle)/360), :]\
                = occluded_surface[int(occluded_surface.shape[0]*(90-front_angle)/360):int(occluded_surface.shape[0]*(90+front_angle)/360), :] * 2
            occluded_surface[int(occluded_surface.shape[0]*(270-back_angle)/360):int(occluded_surface.shape[0]*(270+back_angle)/360), :]\
                = occluded_surface[int(occluded_surface.shape[0]*(270-back_angle)/360):int(occluded_surface.shape[0]*(270+back_angle)/360), :] * 2
        if np_coords.shape[0] > 1:
            occluded_surface = calculate_sensor_occlusion(occluded_surface, np_coords, index)
        total_surface = total_surface + occluded_surface
    if np_coords.shape[0] > 1:
        def redundancy_scaling(x, num_sensors=np_coords.shape[0]):
            # return 1 if x > 0 else 0
            return (2-(0.5)**(x-1))/(2-(0.5)**(num_sensors-1))
        total_surface = np.vectorize(redundancy_scaling)(total_surface)
    # debugVisualizer.debugVisualizer(measure_points)
    return total_surface


def select_random_vehicle(search_params='*vehicle*'):
    global vehicle
    global world
    global bp_lib
    global vehicle_bp
    global all_vehicle_bp
    global test_rectangle_flag
    if test_rectangle_flag:
        test_rectangle_flag = False
        for actor in world.get_actors().filter('box02*'):
            actor.destroy()
        place_cylinder_and_car()
    if not vehicle.destroy():
        for actor in world.get_actors().filter('*vehicle*'): actor.destroy()
    time.sleep(0.1)
    all_vehicle_bp = bp_lib.filter('*vehicle*')
    rand_index = random.randint(0, 20)
    # for some reason, entries 33 and 35 do not exist and cause crashes
    while rand_index == 33 or rand_index == 35:
        rand_index = random.randint(0, len(all_vehicle_bp)-1)
    vehicle = world.try_spawn_actor(all_vehicle_bp[rand_index], carla.Transform(carla.Location(x=0, y=0, z=0.3)))
    if vehicle is None:
        vehicle = world.spawn_actor(all_vehicle_bp[rand_index], carla.Transform(carla.Location(x=0, y=0, z=0.3)))
    return


""" debug methods: """


def create_color_cylinder_surface(sensor_coords):
    placeSensor(np.array([[0.0, 0.0, 2.0]]),  place_sensor_outside_of_vehicle=False)
    measure_points = np.asarray(
        open3d.geometry.PointCloud(open3d.utility.Vector3dVector(fetchLiDARMeasurement())).voxel_down_sample(
            voxel_size=.005).points)
    cylinder_points = cartesian_to_cylindrical_coords(measure_points)
    # debugVisualizer.debugVisualizer(cylinder_points, cylindrical=True, downsample=False)
    tlo_surface = calculate_tlo_with_occlusion(coords=sensor_coords, cylinder_shape=(720, 40), front_and_back_scaling=False)
    colors = np.zeros(shape=measure_points.shape)
    for point in range(cylinder_points.shape[0]):
        if 0 <= cylinder_points[point, 2] <= 4 and cylinder_points[point, 0] > 9.0:
            colors[point] = [0.0, tlo_surface[int(tlo_surface.shape[0]*cylinder_points[point, 1]/(2*math.pi)), int(tlo_surface.shape[1]*(cylinder_points[point, 2]/4.0))], 0.0]
    return measure_points, colors


def find_tlo_of_car_mesh(testBox=True, const_coords=np.zeros(shape=(0, 3))):
    setup_carla_environment(no_render=True, macos=True)
    place_cylinder_and_car()
    color_map = np.array(cm.get_cmap('turbo').colors)
    color_range = np.linspace(0.0, 1.0, color_map.shape[0])
    if testBox:
        points = testBoxMesh(5000, True, True)
    else:
        points = findCarMesh(5000, True, True)
    debugVisualizer.debugVisualizer(points)
    colors = np.zeros(shape=points.shape)
    print("range: ", points.shape[0])
    max_value = 0.0
    for point in range(points.shape[0]):
        start_time = time.thread_time()
        sensor_coord = correct_input_points_to_point_on_vehicle(np.copy(points[point]))
        intensity = calculate_tlo_with_occlusion(np.append(const_coords, sensor_coord, axis=0), cylinder_shape=(720, 20), front_and_back_scaling=False).sum()/720/20
        if intensity > max_value:
            max_value = intensity
        print("point: ", point, " |return : ", intensity, " |time: ", time.thread_time() - start_time)
        colors[point] = np.array([np.interp(intensity, color_range, color_map[:, 0]),
                                  np.interp(intensity, color_range, color_map[:, 1]),
                                  np.interp(intensity, color_range, color_map[:, 2])])
    if const_coords.shape[0] > 0:
        sensor_colors = np.zeros(shape=const_coords.shape)
        sensor_colors[:] = [0.0, 0.0, 1.0]
        points = np.append(points, const_coords, axis=0)
        colors = np.append(colors, sensor_colors, axis=0)
    print(max_value)
    debugVisualizer.debugVisualizer(points, color_list=colors, colors=True, downsample=False)


def find_car_mesh_q_function(testBox=False, load_model_file='model_6000_1_mix'):
    from TensorflowInterface import create_critic
    import tensorflow as tf
    critic_network = create_critic(6000*3, 1*3, load_model_file + '/critic')
    color_map = np.array(cm.get_cmap('turbo').colors)
    color_range = np.linspace(0.0, 1.0, color_map.shape[0])
    if testBox:
        points = testBoxMesh(6000, True, False)
    else:
        points = findCarMesh(6000, True, False)
    # debugVisualizer.debugVisualizer(points)
    colors = np.zeros(shape=points.shape)

    # points = points[points[:, 2].argsort()]
    # points = points[points[:, 1].argsort(kind='mergesort')]
    # points = points[points[:, 0].argsort(kind='mergesort')]
    print("range: ", points.shape[0])
    for point in range(points.shape[0]):
        start_time = time.thread_time()
        sensor_coord = correct_input_points_to_point_on_vehicle(np.copy(points[point]))
        intensity = float(critic_network.call((tf.convert_to_tensor(points.reshape(1, -1)), tf.convert_to_tensor(sensor_coord.reshape(1, -1))), training=False))
        print("point: ", point, " |return : ", intensity, " |time: ", time.thread_time() - start_time)
        colors[point] = np.array([np.interp(intensity, color_range, color_map[:, 0]),
                                  np.interp(intensity, color_range, color_map[:, 1]),
                                  np.interp(intensity, color_range, color_map[:, 2])])
    debugVisualizer.debugVisualizer(points, color_list=colors, colors=True, downsample=False)


def show_color_lidar_measurement(testBox=False, coords=np.array([[0.0, 0.0, 2.0], [0.4, 0.0, 2.0], [0.4*math.sin(7*math.pi/6), 0.4*math.cos(7*math.pi/6), 2.0], [0.4*math.sin(11*math.pi/6), 0.4*math.cos(11*math.pi/6), 2.0]])):
    color_map = np.array(cm.get_cmap('viridis').colors)
    if testBox:
        points = testBoxMesh(5000, True, True)
    else:
        points = findCarMesh(5000, True, True)
    # debugVisualizer.debugVisualizer(points)
    colors = np.zeros(shape=points.shape)
    colored_surface, colors = create_color_cylinder_surface(coords)
    combined_points = np.vstack((points, colored_surface, coords))
    sensor_colors = np.zeros(shape=coords.shape)
    sensor_colors[0] = [0.0, 0.0, 1.0]
    sensor_colors[1:] = [0.0, 0.0, 1.0]
    combined_colors = np.vstack((np.full(shape=points.shape, fill_value=0.7), colors, sensor_colors))
    color_cloud = open3d.geometry.PointCloud(open3d.utility.Vector3dVector(combined_points))
    color_cloud.colors = open3d.utility.Vector3dVector(combined_colors)
    debugVisualizer.debugVisualizer(color_cloud, colors=False, downsample=False)


def closeEnvironment(macos=False):
    global world
    global carlaClientApp
    for actor in world.get_actors().filter('*vehicle*'):
        actor.destroy()
    for actor in world.get_actors().filter('*sensor*'):
        actor.stop()
        actor.destroy()
    if macos:
        os.system("killall CarlaUE4-Mac-Shipping")


if __name__ == '__main__':
    try:
        coords = np.array([
            [0.0, 0.0, 1.9],
            [-1.5, 0.0, 1.9]
        ])
        """
        setup_carla_environment(macos=True)
        place_cylinder_and_car()
        findCarMesh(6000, True, False)
        #testBoxMesh(setNumber=True)
        find_car_mesh_q_function(testBox=False, load_model_file='model_1_rand')
        show_color_lidar_measurement(testBox=False, coords=coords)
        print(calculate_tlo_with_occlusion(coords=coords, cylinder_shape=(720, 40), front_and_back_scaling=False).sum()/720/40)
        """
        # find_tlo_of_car_mesh(False, const_coords=coords)  # """
    finally:
        closeEnvironment(macos=True)
