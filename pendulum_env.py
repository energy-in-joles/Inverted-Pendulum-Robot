import numpy as np
from serial import Serial
from gym import Env
from gym.spaces import Discrete, Box
from omegaconf import DictConfig
import time

class PendulumEnv(Env):
    def __init__(self, cfg: DictConfig, ser: Serial):
        super(PendulumEnv, self).__init__()
        self.ser = ser
        self.serial_cfg = cfg.serial
        self.action_half_range = cfg.action_space.half_range
        self.action_interval = cfg.action_space.interval
        self.action_space = Discrete(self.action_half_range * 2 // self.action_interval)
        self._setup()

    def step(self):
        pass

    def reset(self):
        pass

    def _setup(self) -> None:
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
        isReady = False
        line = ""
        start_t = time.time()
        startup_t = 0

        while not (isReady or startup_t >= self.serial_cfg.max_startup_t):
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').rstrip()
            if line == self.serial_cfg.auth_str:
                isReady = True
            startup_t = time.time() - start_t

        if not isReady:
            raise Exception("Arduino maximum setup time exceeded!")

        print("-----------------")
        print("Arduino is ready!")
        print("-----------------\n")

        # send initial accel of 0 to start the data exchange loop
        self.ser.write(INI_BYTE_STR)
    
    # decode action in discrete space to actual acceleration value
    def action_to_acceleration(self, action: int):
        return action * self.action_interval - self.action_half_range

