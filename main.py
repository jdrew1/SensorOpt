import time
import open3d
import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt

import CarlaInterface
import CarlaWrapper
import TensorflowInterface
import debugVisualizer


def run_test(macos=False, load_trained_model="model_6000_1_mix/", save_dir="model_6000_1_mix/",
             train_networks=True):
    env = CarlaWrapper.CarlaWrapperEnv(
        macos=macos,
        numOfPoints=6000,
        numOfLiDAR=1,
        no_render=False,
        test_box=False,
        cylinder_shape=(720, 20),
        down_sample_lidar=True,
        front_and_back_scaling=False,
        shuffle_vehicles=True
    )
    place_sensor_on_surface = True

    # Learning rate for actor-critic models
    critic_lr = 0.002
    actor_lr = 0.001

    total_episodes = 100
    iterations_per_episode = 2
    buffer_length = 5000
    training_batch_size = 64
    # target network update rate
    rho = 0.995

    print("Size of State Space ->  {}".format(env.observation_spec.shape[0]))
    print("Size of Action Space ->  {}".format(env.action_spec.shape[0]))

    # start noise high for better searching, then decrease to hone
    def std_dev(step):
        # return 0.02
        return float(1.0 / (step/20 + 1)) * np.ones(env.action_spec.shape[0])
    ou_noise = TensorflowInterface.OUActionNoise(mean=np.zeros(env.action_spec.shape[0]), std_deviation_function=std_dev)

    actor_model = TensorflowInterface.create_actor(env.observation_spec.shape[0], env.action_spec.shape[0],
                                                   env.action_spec.high, env.action_spec.low,
                                                   (load_trained_model + 'actor') if load_trained_model != '' else '')
    critic_model = TensorflowInterface.create_critic(env.observation_spec.shape[0], env.action_spec.shape[0],
                                                     (load_trained_model + 'critic') if load_trained_model != '' else '')

    target_actor = TensorflowInterface.create_actor(env.observation_spec.shape[0], env.action_spec.shape[0],
                                                    env.action_spec.high, env.action_spec.low,
                                                    (load_trained_model + 'target_actor') if load_trained_model != '' else '')
    target_critic = TensorflowInterface.create_critic(env.observation_spec.shape[0], env.action_spec.shape[0],
                                                      (load_trained_model + 'target_critic') if load_trained_model != '' else '')

    # to initialize, target = learning
    if load_trained_model == '':
        target_actor.set_weights(actor_model.get_weights())
        target_critic.set_weights(critic_model.get_weights())

    critic_optimizer = tf.keras.optimizers.Adam(critic_lr)
    actor_optimizer = tf.keras.optimizers.Adam(actor_lr)

    buffer = TensorflowInterface.Buffer(actor_model, critic_model, actor_optimizer, critic_optimizer, target_actor, target_critic,
                                        buffer_length, iterations_per_episode*training_batch_size, num_lidar=env.action_spec.shape[0])

    # store list of rewards in more convienent format than buffer
    ep_reward_list = []
    # rolling average to show change over time
    avg_reward_list = []

    for ep in range(total_episodes):
        start_time = time.thread_time()
        prev_state = env.reset()
        episodic_reward = 0

        for i in range(iterations_per_episode):
            tf_prev_state = tf.expand_dims(tf.convert_to_tensor(prev_state), 0)
            # generate action with noise
            action = TensorflowInterface.policy_with_noise(tf_prev_state, ou_noise, actor_model,
                                                           env.action_spec.low, env.action_spec.high,
                                                           ep*iterations_per_episode, place_sensor_on_surface)
            state, reward, done, info = env.step(action)
            # record experience to buffer
            buffer.record((prev_state, action, reward, state))
            episodic_reward += reward
            # train from collected experience
            if train_networks:
                buffer.learn()
                TensorflowInterface.update_target(target_actor.variables, actor_model.variables, rho)
                TensorflowInterface.update_target(target_critic.variables, critic_model.variables, rho)

            prev_state = state

        episodic_reward = episodic_reward / iterations_per_episode

        ep_reward_list.append(episodic_reward)

        # Mean of last 10 episodes
        avg_reward = np.mean(ep_reward_list[-20:])
        print("Episode * {} * Action {}* Avg Reward is ==> {}".format(ep, action, avg_reward))
        print("Time elapse: ", time.thread_time() - start_time)
        avg_reward_list.append(avg_reward)

    # Plotting graph
    # Episodes versus Avg. Rewards
    show_training_path(coords_replay=np.copy(buffer.action_buffer[:(buffer.buffer_counter+1) % buffer.buffer_capacity, :]))
    plt.plot(avg_reward_list)
    plt.xlabel("")
    plt.ylabel("Average Lidar Occupancy")
    plt.ylim(0, 1)
    plt.show()

    # Save the weights

    actor_model.compile()
    critic_model.compile()
    target_actor.compile()
    target_critic.compile()

    actor_model.save(save_dir + "actor")
    critic_model.save(save_dir + "critic")
    target_actor.save(save_dir + "target_actor")
    target_critic.save(save_dir + "target_critic")


def show_training_path(coords_replay):
    if coords_replay.shape[1] != 3:
        coords_replay = coords_replay.reshape((-1, 3))
    gradient = np.linspace([1.0, 0.1, 0.1], [0.0, 0.0, 0.0], num=coords_replay.shape[0])
    color_cloud = open3d.geometry.PointCloud(open3d.utility.Vector3dVector(coords_replay))
    color_cloud.colors = open3d.utility.Vector3dVector(gradient)
    debugVisualizer.debugVisualizer(color_cloud, colors=False, downsample=False)
    return


if __name__ == '__main__':
    try:
        open_and_close_automatically = True
        run_test(macos=open_and_close_automatically)
        # debugMethods:
        # CarlaInterface.show_color_lidar_measurement(testBox=False)
        # CarlaInterface.find_tlo_of_car_mesh(testBox=False)
    finally:
        CarlaInterface.closeEnvironment(macos=open_and_close_automatically)
