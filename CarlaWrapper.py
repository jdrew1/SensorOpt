import numpy as np
import time
import gym
from gym import spaces
import CarlaInterface
import debugVisualizer


class CarlaWrapperEnv(gym.Env):

    def __init__(self, numOfPoints=6000, numOfLiDAR=1, no_render=False, test_box=True, cylinder_shape=(360, 20),
                 down_sample_lidar=True, front_and_back_scaling=True, shuffle_vehicles=False, macos=False):
        CarlaInterface.setup_carla_environment(no_render=no_render, macos=macos)
        CarlaInterface.place_cylinder_and_car(distance=10.0)
        self.test_box = test_box
        self.num_points = numOfPoints
        self.num_lidar = numOfLiDAR
        outputshape = [-4.0, -4.0, 0.5]
        for i in range(numOfLiDAR-1):
            outputshape = np.append(outputshape, np.asarray([-4.0, -4.0, 0.5]))
        self.min_action = np.copy(outputshape)
        outputshape = [4.0, 4.0, 3.5]
        for i in range(numOfLiDAR-1):
            outputshape = np.append(outputshape, np.asarray([4.0, 4.0, 3.5]))
        self.max_action = np.copy(outputshape)
        self.action_spec = spaces.Box(low=self.min_action, high=self.max_action, shape=(numOfLiDAR*3,))
        self.observation_spec = spaces.Box(low=-200, high=200, shape=(numOfPoints*3,))

        self.episode_finished = False  # state = number of lidar being placed

        self._sensor_placements = np.zeros((numOfLiDAR,), dtype=float)
        self._vehiclePoints = np.zeros((numOfPoints, 3), dtype=float)
        self.cylinder_shape = cylinder_shape
        self.down_sample_lidar = down_sample_lidar
        self.front_and_back_scaling = front_and_back_scaling
        self.shuffle_vehicle = shuffle_vehicles
        self.cumulative_TLO = 0.0

    def reset(self):
        self.cumulative_TLO = 0.0
        if self.test_box:
            self._vehiclePoints = CarlaInterface.testBoxMesh(self.num_points, True).flatten()
        else:
            self._vehiclePoints = CarlaInterface.findCarMesh(self.num_points, True).flatten()
        return self._vehiclePoints.flatten()

    def step(self, action):
        info = {}
        self._sensor_placements = np.asarray(action).flatten()
        # collect points
        if self.test_box:
            self._vehiclePoints = CarlaInterface.testBoxMesh(self.num_points, True)
        else:
            if self.shuffle_vehicle:
                CarlaInterface.select_random_vehicle()
            self._vehiclePoints = CarlaInterface.findCarMesh(self.num_points, True)
        self._vehiclePoints = self._vehiclePoints[self._vehiclePoints[:, 2].argsort()]
        self._vehiclePoints = self._vehiclePoints[self._vehiclePoints[:, 1].argsort(kind='mergesort')]
        self._vehiclePoints = self._vehiclePoints[self._vehiclePoints[:, 0].argsort(kind='mergesort')].flatten()
        # debugVisualizer.debugVisualizer(self._vehiclePoints)
        self.cumulative_TLO = CarlaInterface.calculate_tlo_with_occlusion(self._sensor_placements,
                                                                          cylinder_shape=self.cylinder_shape,
                                                                          downsample=self.down_sample_lidar,
                                                                          front_and_back_scaling=self.front_and_back_scaling)\
                                            .sum()
        self.cumulative_TLO = self.cumulative_TLO/self.cylinder_shape[0]/self.cylinder_shape[1]
        self.episode_finished = True
        return self._vehiclePoints, self.cumulative_TLO, self.episode_finished, info