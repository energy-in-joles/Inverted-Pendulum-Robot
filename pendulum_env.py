import numpy as np
from numpy.typing import NDArray
from math import cos, sin
from serial import Serial
from gym import Env
from gym.spaces import Discrete, Box
from omegaconf import DictConfig
import time

from util import PosInfo

from util import (
    interpret_encoder_info,
    get_theta,
    calculate_velocity,
    calculate_acceleration,
    normalise_motor_pos,
    calculate_reward
)

# states: 0 = ready/training, 1 = done, 2 = resetting

class PendulumEnv(Env):
    def __init__(self, cfg: DictConfig, ser: Serial):
        super(PendulumEnv, self).__init__()
        self.cfg = cfg
        self.is_done = False
        self.ser = ser

        self.action_space = self._create_action_space(cfg.action_space)
        self.observation_space = self._create_observation_space(cfg.observation_space)

        self.episode_frame = 0
        self.last_t = 0
        self.last_pos_info = None
        self.last_vel = 0
        self.last_observation = None

        self._setup(print_ready=True)
    
    # move robot based on given action and feedback environment data to decision maker
    def step(self, action: int) -> tuple[NDArray, float, bool, bool, dict]:
        accel = self._action_to_acceleration(action)
        print(accel)
        self.ser.write(accel.to_bytes(self.cfg.serial.out_buffer_size, byteorder='little', signed=True))
        
        data_in = self.ser.read(self.cfg.serial.in_buffer_size)
        this_t = time.time()

        pos, loop_i, motor_pos = interpret_encoder_info(data_in)
        this_pos_info = PosInfo(pos, loop_i, motor_pos)
        this_theta = get_theta(this_pos_info.pos)
        observation = self._create_observation_array(this_t - self.last_t, this_pos_info, this_theta)
        reward = calculate_reward(
            this_theta, observation[2], observation[3], observation[4], self.cfg.reward.vel_weight, self.cfg.reward.accel_weight
            )
        
        if abs(motor_pos) >= self.cfg.calc.motor_half_limit:
            terminated = True
        else:
            terminated = False

        if self.episode_frame >= self.cfg.episode.episode_length:
            truncated = True
        else:
            truncated = False

        info = {}
        self.last_t = this_t
        self.last_pos_info = this_pos_info
        self.last_vel = observation[2]
        self.episode_frame += 1
        return observation, reward, terminated, truncated, info
    
    # reset environment back to starting position
    def reset(self):
        self._reset_pos()
        self.episode_frame = 0
        self._setup()
        time.sleep(self.cfg.serial.sleep_t)

    def _setup(self, print_ready=False) -> None:
        """
        Initializes communication with an Arduino over a serial connection.

        Args:
            ser (serial.Serial): The serial object for communication with the Arduino.
            max_startup_time (int, optional): Maximum time to wait for the Arduino to be ready, in seconds. Default is 5 seconds.
            auth_str (str, optional): The authentication string sent by the Arduino to indicate it is ready. Default is "<ready>".

        Raises:
            Exception: If the Arduino does not respond with the authentication string within the allowed startup time.
        """
        is_ready = False
        line = ""
        start_t = time.time()
        startup_t = 0

        while not (is_ready or startup_t >= self.cfg.serial.max_startup_t):
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').rstrip()
            if line == self.cfg.serial.auth_str:
                is_ready = True
            startup_t = time.time() - start_t

        if not is_ready:
            raise Exception("Arduino maximum setup time exceeded!")
        
        if print_ready:
            print("-----------------")
            print("Arduino is ready!")
            print("-----------------\n")

        self._update_last_pos_vel()

    def _reset_pos(self) -> None:
        RESET_CMD = 32767 # out of bounds from highest acceleration value
        self.ser.write(RESET_CMD.to_bytes(self.cfg.serial.out_buffer_size, byteorder='little', signed=True))

    # update last pos and last vel info at the start of an episode (we can't get our first observation without these)
    def _update_last_pos_vel(self) -> None:
        ZERO_ACCEL_STR = b'\x00' * self.cfg.serial.out_buffer_size
        
        # first episode frame: update only last_pos_info and last_t
        self.ser.write(ZERO_ACCEL_STR)
        data_in = self.ser.read(self.cfg.serial.in_buffer_size)
        self.last_t = time.time()
        pos, loop_i, motor_pos = interpret_encoder_info(data_in)
        self.last_pos_info = PosInfo(pos, loop_i, motor_pos)

        #escond episode frame: update last_pos_info, last_t and last_vel
        self.ser.write(ZERO_ACCEL_STR)
        data_in = self.ser.read(self.cfg.serial.in_buffer_size)
        this_t = time.time()
        pos, loop_i, motor_pos = interpret_encoder_info(data_in)
        this_pos_info = PosInfo(pos, loop_i, motor_pos)
        this_vel = calculate_velocity(this_t - self.last_t, self.last_pos_info, this_pos_info, self.cfg.calc.vel_modifier)
        self.last_t = this_t
        self.last_pos_info = this_pos_info
        self.last_vel = this_vel

    # decode action in discrete space to actual acceleration value
    def _action_to_acceleration(self, action: int) -> int:
        return action * self.cfg.action_space.interval - self.cfg.action_space.half_range
    
    def _create_action_space(self, action_space_cfg: DictConfig):
        return Discrete(action_space_cfg.half_range * 2 // action_space_cfg.interval)

    # generate Box object observation space
    def _create_observation_space(self, obs_space_cfg: DictConfig) -> Box:
        lower_bound = np.array(obs_space_cfg.lower_bound, dtype=np.float32)
        upper_bound = np.array(obs_space_cfg.upper_bound, dtype=np.float32)
        return Box(low=lower_bound, high=upper_bound, dtype=np.float32)
    
    # wrap observations into array that can be fed into the neural network
    def _create_observation_array(self, delta_t: float, this_pos_info: PosInfo, this_theta: float) -> NDArray[np.int32]:
        this_vel = calculate_velocity(delta_t, self.last_pos_info, this_pos_info, self.cfg.calc.vel_modifier)
        this_accel = calculate_acceleration(delta_t, self.last_vel, this_vel, self.cfg.calc.accel_modifier)
        this_motor_pos = normalise_motor_pos(this_pos_info.motor_pos, self.cfg.calc.motor_half_range)
        x = cos(this_theta)
        y = sin(this_theta)
        return np.array([x, y, this_vel, this_accel, this_motor_pos], dtype=np.float32)