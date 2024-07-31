import numpy as np
from numpy.typing import NDArray
from math import cos, sin
from serial import Serial
from gym import Env
from gym.spaces import Discrete, Box
from omegaconf import DictConfig
import time

class PendulumEnv(Env):
    def __init__(self, cfg: DictConfig, ser: Serial):
        super(PendulumEnv, self).__init__()
        self.ser = ser
        self.action_half_range = cfg.action_space.half_range
        self.action_interval = cfg.action_space.interval
        self.action_space = Discrete((self.action_half_range * 2 // self.action_interval) + 1)
        self.observation_space = self._create_observation_space(cfg.observation_space)
        self._setup(cfg.serial)

    def step(self):
        pass

    def reset(self):
        pass

    def _setup(self, serial_cfg: DictConfig) -> None:
        """
        Initializes communication with an Arduino over a serial connection.

        Args:
            ser (serial.Serial): The serial object for communication with the Arduino.
            max_startup_time (int, optional): Maximum time to wait for the Arduino to be ready, in seconds. Default is 5 seconds.
            auth_str (str, optional): The authentication string sent by the Arduino to indicate it is ready. Default is "<ready>".

        Raises:
            Exception: If the Arduino does not respond with the authentication string within the allowed startup time.
        """

        INI_BYTE_STR = b'\x00\x00'
        is_ready = False
        line = ""
        start_t = time.time()
        startup_t = 0

        while not (is_ready or startup_t >= serial_cfg.max_startup_t):
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').rstrip()
            if line == serial_cfg.auth_str:
                is_ready = True
            startup_t = time.time() - start_t

        if not is_ready:
            raise Exception("Arduino maximum setup time exceeded!")

        print("-----------------")
        print("Arduino is ready!")
        print("-----------------\n")

        # send initial accel of 0 to start the data exchange loop
        self.ser.write(INI_BYTE_STR)
    
    # decode action in discrete space to actual acceleration value
    def _action_to_acceleration(self, action: int) -> int:
        return action * self.action_interval - self.action_half_range

    # reward function following pendulum environment in openai
    def _calculate_reward(self, theta: float, vel: float, accel: float, vel_weight: float, accel_weight: float) -> float:
        return -(theta ** 2 + 0.1 * vel_weight * (vel ** 2) + accel_weight * (accel ** 2))
    
    # generate Box object observation space
    def _create_observation_space(self, obs_space_cfg: DictConfig) -> Box:
        lower_bound = np.array(obs_space_cfg.lower_bound, dtype=np.float32)
        upper_bound = np.array(obs_space_cfg.upper_bound, dtype=np.float32)
        return Box(low=lower_bound, high=upper_bound, dtype=np.float32)
    
    # wrap observations into array that can be fed into the neural network
    def _create_observation_array(self, theta: float, vel: float) -> NDArray[np.int32]:
        x = cos(theta)
        y = sin(theta)
        return np.array([x, y, vel], dtype=np.float32)