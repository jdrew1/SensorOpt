import tensorflow as tf
from tensorflow import keras
from keras import layers
import numpy as np
from CarlaInterface import correct_input_points_to_point_on_vehicle


def policy_with_noise(state, noise_object, actor_model, lower_bound, upper_bound, step, place_sensor_on_vehicle=True, random=False):
    if random:
        sampled_actions = tf.squeeze(np.random.uniform(low=[-3.0, -3.0, 0.5], high=[3.0, 3.0, 3.5], size=3))
        legal_action = np.clip(sampled_actions, lower_bound, upper_bound)
        if place_sensor_on_vehicle:
            legal_action = correct_input_points_to_point_on_vehicle(legal_action).flatten()
        return np.squeeze(legal_action)
    else:
        sampled_actions = tf.squeeze(actor_model(state))
    noise = noise_object(step)
    # add noise to help search
    sampled_actions = sampled_actions.numpy() + noise
    # check for legal action and correct to be on vehicle
    legal_action = np.clip(sampled_actions, lower_bound, upper_bound)
    if place_sensor_on_vehicle:
        legal_action = correct_input_points_to_point_on_vehicle(legal_action).flatten()
    return np.squeeze(legal_action)


def create_actor(num_points, num_lidar, upper_bound, lower_bound, load_from_file=''):
    if load_from_file != '':
        model = keras.models.load_model(load_from_file)
        model.compile()
        return model
    last_init = tf.random_uniform_initializer(minval=-0.003, maxval=0.003)
    inputs = layers.Input(shape=(num_points,))
    out = layers.Flatten()(inputs)
    out = layers.Dense(2048, activation="relu")(out)
    out = layers.Dense(4092, activation="relu")(out)
    out = layers.Dense(512, activation="relu")(out)
    out = layers.Dense(512, activation="relu")(out)
    out = layers.Dense(256, activation="relu")(out)
    outputs = layers.Dense(num_lidar, activation="sigmoid", kernel_initializer=last_init)(out)
    output_space_span = upper_bound - lower_bound
    outputs = ((outputs * output_space_span) + lower_bound)
    model = tf.keras.Model(inputs, outputs)
    model.compile()
    return model


def create_critic(num_points, num_lidar, load_from_file=''):
    if load_from_file != '':
        model = keras.models.load_model(load_from_file)
        model.compile()
        return model
    state_input = layers.Input(shape=(num_points,))
    state_out = layers.Dense(1024, activation="linear")(state_input)
    state_out = layers.Dense(512, activation="relu")(state_out)
    action_input = layers.Input(shape=(num_lidar,))
    action_out = layers.Dense(256, activation="linear")(action_input)
    action_out = layers.Dense(512, activation="relu")(action_out)
    concat = layers.Concatenate()([state_out, action_out])
    out = layers.Dense(1024, activation="tanh")(concat)
    out = layers.Dense(512, activation="tanh")(out)
    out = layers.Dense(128, activation="tanh")(out)
    out = layers.Dense(32, activation="tanh")(out)
    outputs = layers.Dense(1, activation="sigmoid")(out)
    model = tf.keras.Model([state_input, action_input], outputs)
    model.compile()
    return model


@tf.function
def update_target(target_weights, weights, rho):
    for (a, b) in zip(target_weights, weights):
        a.assign(a * rho + b * (1 - rho))


# Ornstein-Uhlenbeck noise to add randomness to training
class OUActionNoise:
    def __init__(self, mean, std_deviation_function, theta=0.15, dt=1e-2, x_initial=None):
        self.theta = theta
        self.mean = mean
        self.std_dev = lambda step: std_deviation_function(step)
        self.dt = dt
        self.x_initial = x_initial
        self.x_prev = np.zeros_like(self.mean)
        self.reset()

    def __call__(self, step):
        x = np.random.normal(size=self.mean.shape, scale=self.std_dev(step))
        self.x_prev = x
        return x

    def reset(self):
        if self.x_initial is not None:
            self.x_prev = self.x_initial
        else:
            self.x_prev = np.zeros_like(self.mean)


# experience replay buffer
class Buffer:
    def __init__(self, actor_model, critic_model, actor_optimizer, critic_optimizer, target_actor, target_critic,
                 buffer_capacity=100000, batch_size=32, num_points=18000, num_lidar=3, rho=0.99):
        self.buffer_capacity = buffer_capacity
        self.batch_size = batch_size
        self.buffer_counter = 0
        self.rho = rho

        self.state_buffer = np.zeros((self.buffer_capacity, num_points))
        self.action_buffer = np.zeros((self.buffer_capacity, num_lidar))
        self.reward_buffer = np.zeros((self.buffer_capacity, 1))
        self.next_state_buffer = np.zeros((self.buffer_capacity, num_points))

        self.actor_model = actor_model
        self.critic_model = critic_model
        self.target_actor = target_actor
        self.target_critic = target_critic
        self.actor_optimizer = actor_optimizer
        self.critic_optimizer = critic_optimizer

    # Takes (s,a,r,s') observation as input
    def record(self, obs_tuple):
        index = self.buffer_counter % self.buffer_capacity
        self.state_buffer[index] = obs_tuple[0]
        self.action_buffer[index] = obs_tuple[1]
        self.reward_buffer[index] = obs_tuple[2]
        self.next_state_buffer[index] = obs_tuple[3]

        self.buffer_counter += 1

    # package this function as tensorflow to allow for tensorflow optimization/parallelization
    @tf.function
    def update(self, state_batch, action_batch, reward_batch, next_state_batch, ):
        # use gradient tape to find training direction for actor and critic
        with tf.GradientTape() as tape:
            target_actions = self.target_actor(next_state_batch, training=True)
            y = reward_batch  # + (1 - self.rho) * self.target_critic([next_state_batch, target_actions], training=True)
            critic_value = self.critic_model([state_batch, action_batch], training=True)
            critic_loss = tf.math.reduce_mean(tf.math.square(y - critic_value))
        print("Reward batch: ->")
        tf.print(reward_batch)
        print("Critic batch: ->")
        tf.print(critic_value)
        print("Current Critic Loss: -> ")
        tf.print(critic_loss)
        critic_grad = tape.gradient(critic_loss, self.critic_model.trainable_variables)
        self.critic_optimizer.apply_gradients(zip(critic_grad, self.critic_model.trainable_variables))
        with tf.GradientTape() as tape:
            actions = self.actor_model(state_batch, training=True)
            critic_value = self.critic_model([state_batch, actions], training=True)
            actor_loss = -tf.math.reduce_mean(critic_value)
        actor_grad = tape.gradient(actor_loss, self.actor_model.trainable_variables)
        self.actor_optimizer.apply_gradients(zip(actor_grad, self.actor_model.trainable_variables))

    # compute loss and update weights and biases
    def learn(self):
        record_range = min(self.buffer_counter, self.buffer_capacity)
        batch_indices = np.random.choice(record_range, min(self.batch_size, record_range))
        state_batch = tf.convert_to_tensor(self.state_buffer[batch_indices])
        action_batch = tf.convert_to_tensor(self.action_buffer[batch_indices])
        reward_batch = tf.convert_to_tensor(self.reward_buffer[batch_indices])
        reward_batch = tf.cast(reward_batch, dtype=tf.float32)
        next_state_batch = tf.convert_to_tensor(self.next_state_buffer[batch_indices])
        self.update(state_batch, action_batch, reward_batch, next_state_batch)
