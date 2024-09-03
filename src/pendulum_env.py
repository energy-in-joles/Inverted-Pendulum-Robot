import numpy as np
from numpy.typing import NDArray
from typing import Any
from serial import Serial
from gymnasium import Env
from gymnasium.spaces import Box
from omegaconf import DictConfig
import time

from util import PosInfo

from util import (
    interpret_encoder_info,
    get_theta,
    calculate_velocity,
    normalise_motor_pos,
    calculate_norm_motor_velocity,
)


class PendulumEnv(Env):
    def __init__(self, cfg: DictConfig, ser: Serial):
        super(PendulumEnv, self).__init__()
        self.cfg = cfg
        self.is_done = False
        self.ser = ser

        self.action_space = self._create_action_space(cfg.action_space)
        self.observation_space = self._create_observation_space(cfg.observation_space)
        self.episode_length = cfg.mode.episode_length
        self.episode_frame = 0
        self.last_t = 0
        self.last_pos_info = None
        self.step_interval = 1 / cfg.control_frequency
        self.last_step_t = 0

        self._setup(print_ready=True)

    # move robot based on given action and feedback environment data to decision maker
    def step(self, action: NDArray) -> tuple[NDArray, float, bool, bool, dict]:
        self._enforce_control_frequency()

        accel = self._action_to_acceleration(action[0])
        self.ser.write(
            accel.to_bytes(
                self.cfg.serial.out_buffer_size, byteorder="little", signed=True
            )
        )

        data_in = self.ser.read(self.cfg.serial.in_buffer_size)
        this_t = time.time()

        pos, loop_i, motor_pos = interpret_encoder_info(data_in)
        this_pos_info = PosInfo(pos, loop_i, motor_pos)
        this_theta = get_theta(this_pos_info.pos)
        observation = self._create_observation_array(
            this_t - self.last_t, this_theta, this_pos_info
        )

        if abs(motor_pos) >= self.cfg.calc.motor_half_limit:
            terminated = True
        else:
            terminated = False

        # episode length == -1 means infinite episode length: used for eval
        if self.episode_length != -1 and self.episode_frame >= self.episode_length - 1:
            truncated = True
        else:
            truncated = False

        reward = self._calculate_reward(
            this_theta, observation[2], observation[3], observation[4], action
        )
        info = {}
        self.last_t = this_t
        self.last_pos_info = this_pos_info
        self.episode_frame += 1
        return observation, reward, terminated, truncated, info

    def close(self):
        self.reset()

    # reset environment back to starting position
    def reset(self, seed: int = None) -> tuple[NDArray, dict[str, Any]]:
        super().reset(seed=seed)
        self._reset_pos()
        self.episode_frame = 0
        self._setup()

        # if encoder is going to overflow, set a long reset time for pendulum to stop moving then reset encoder
        if abs(self.last_pos_info.loop_i) > self.cfg.encoder.encoder_reset_thresh:
            time.sleep(self.cfg.encoder.reset_encoder_t)
            self._reset_encoder()

        time.sleep(self.cfg.serial.sleep_t)
        observation, _, _, _, info = self.step(np.array([0]))
        return observation, info

    def _setup(self, print_ready=False) -> None:
        """
        Initializes communication with an Arduino over a serial connection.

        Args:
            ser (serial.Serial): The serial object for communication with the Arduino.
            print_ready: toggle printing "arduino is ready" in terminal.

        Raises:
            Exception: If the Arduino does not respond with the authentication string within the allowed startup time.
        """
        is_ready = False
        line = ""
        start_t = time.time()
        startup_t = 0

        while not (is_ready or startup_t >= self.cfg.serial.max_startup_t):
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode("utf-8").rstrip()
            if line == self.cfg.serial.auth_str:
                is_ready = True
            startup_t = time.time() - start_t

        if not is_ready:
            raise Exception("Arduino maximum setup time exceeded!")

        if print_ready:
            print("-----------------")
            print("Arduino is ready!")
            print("-----------------\n")

        self._update_first_pos()

    # send reset command to robot to send stepper to position 0
    def _reset_pos(self) -> None:
        RESET_CMD = 32767  # out of bounds from highest acceleration value
        self.ser.write(
            RESET_CMD.to_bytes(
                self.cfg.serial.out_buffer_size, byteorder="little", signed=True
            )
        )

    # send reset command to clear loop_i in encoder
    def _reset_encoder(self) -> None:
        RESET_CMD = 32766
        self.ser.write(
            RESET_CMD.to_bytes(
                self.cfg.serial.out_buffer_size, byteorder="little", signed=True
            )
        )

    # update last pos and time info at the start of an episode (we can't get our first observation without these)
    def _update_first_pos(self) -> None:
        ZERO_ACCEL_STR = b"\x00" * self.cfg.serial.out_buffer_size

        # first episode frame: update only last_pos_info and last_t
        self.ser.write(ZERO_ACCEL_STR)
        data_in = self.ser.read(self.cfg.serial.in_buffer_size)
        self.last_t = time.time()
        pos, loop_i, motor_pos = interpret_encoder_info(data_in)
        self.last_pos_info = PosInfo(pos, loop_i, motor_pos)

    # decode action in normalised continuous action space to actual acceleration value
    def _action_to_acceleration(self, action: float) -> int:
        mult = self.cfg.action_space.actual_half_range / (
            self.cfg.action_space.half_range**3
        )
        return round(action**3 * mult)

    # create continuous Box action space
    def _create_action_space(self, action_space_cfg: DictConfig):
        half_range = action_space_cfg.half_range
        return Box(low=-half_range, high=half_range, shape=(1,), dtype=np.float32)

    # generate continuous Box observation space
    def _create_observation_space(self, obs_space_cfg: DictConfig) -> Box:
        lower_bound = np.array(obs_space_cfg.lower_bound, dtype=np.float32)
        upper_bound = np.array(obs_space_cfg.upper_bound, dtype=np.float32)
        return Box(low=lower_bound, high=upper_bound, dtype=np.float32)

    # wrap observations into array that can be fed into the neural network
    def _create_observation_array(
        self, delta_t: float, this_theta: float, this_pos_info: PosInfo
    ) -> NDArray[np.int32]:
        cos_theta = np.cos(this_theta)
        sin_theta = np.sin(this_theta)

        this_vel = calculate_velocity(
            delta_t, self.last_pos_info, this_pos_info, self.cfg.calc.vel_modifier
        )

        this_motor_pos = normalise_motor_pos(
            this_pos_info.motor_pos,
            self.cfg.calc.motor_half_range,
            self.cfg.calc.motor_norm_half_range,
        )
        last_motor_pos = normalise_motor_pos(
            self.last_pos_info.motor_pos,
            self.cfg.calc.motor_half_range,
            self.cfg.calc.motor_norm_half_range,
        )
        this_norm_motor_vel = calculate_norm_motor_velocity(
            delta_t, last_motor_pos, this_motor_pos, self.cfg.calc.motor_vel_modifier
        )

        return np.array(
            [cos_theta, sin_theta, this_vel, this_motor_pos, this_norm_motor_vel],
            dtype=np.float32,
        )

    # reward function based on Quanser Qube design
    # includes theta, angular velocity, motor_pos, motor_velocity, action
    def _calculate_reward(
        self,
        theta: float,
        vel: float,
        norm_motor_pos: float,
        norm_motor_vel: float,
        action: float,
    ) -> float:
        vel_weight = self.cfg.reward.vel_weight
        motor_pos_weight = self.cfg.reward.motor_pos_weight
        motor_vel_weight = self.cfg.reward.motor_vel_weight
        control_weight = self.cfg.reward.motor_vel_weight
        reward_offset = self.cfg.reward.reward_offset

        cost = (
            (theta) ** 2
            + vel_weight * (vel**2)
            + motor_pos_weight * (norm_motor_pos**2)
            + motor_vel_weight * (norm_motor_vel**2)
            + control_weight * (action**2)
        )

        return reward_offset - cost

    # enforce fixed control frequency of around 55Hz to ensure that robot behaviour is consistent
    # introduce delays if frequency exceeds (happens esp at start of training cycle)
    def _enforce_control_frequency(self):
        now = time.time()
        last_t = self.last_step_t
        self.last_step_t = now
        if last_t != 0 and now - last_t < self.step_interval:
            time.sleep(self.step_interval - (now - last_t))
