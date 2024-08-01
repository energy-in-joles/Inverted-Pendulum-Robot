from serial import Serial
import time
import threading
from omegaconf import DictConfig

from util import PosInfo

from util import (
    interpret_encoder_info,
    calculate_velocity,
    calculate_acceleration,
    calculate_reward,
    get_theta
)

from pendulum_env import PendulumEnv

task_lock = threading.Lock()

def train_model(cfg: DictConfig, ser: Serial, iterations=100000):
    accel = 0
    i = 0
    reward_sum = 0
    FIRST_TWO_FRAMES = 2
    iterations += FIRST_TWO_FRAMES # first 2 iterations no training: not enough data

    pendulumEnv = PendulumEnv(cfg, ser)

    while i < iterations:
        data_in = ser.read(4)
        this_t = time.time()

        pos, loop_i, motor_pos = interpret_encoder_info(data_in)
        this_pos_info = PosInfo(pos, loop_i)
        this_theta = get_theta(this_pos_info.pos)

        # at earlier iterations, data insufficient to calculate vel/accel so we skip training
        if i > 0:
            delta_t = this_t - last_t
            this_vel = calculate_velocity(delta_t, last_pos_info, this_pos_info, cfg.calc.vel_modifier)
            if i > 1:
                this_accel = calculate_acceleration(delta_t, last_vel, this_vel, cfg.calc.accel_modifier)
                print(calculate_reward(this_theta, this_vel, this_accel, cfg.reward.vel_weight, cfg.reward.accel_weight), this_theta, this_vel, this_accel, motor_pos, this_pos_info.pos, this_pos_info.loop_i)
                reward_sum += calculate_reward(this_theta, this_vel, this_accel, cfg.reward.vel_weight, cfg.reward.accel_weight)
                # t1 = threading.Thread(target=test).start()
            last_vel = this_vel
        
        if i > 10:
            pendulumEnv.reset_pos()
        

        time.sleep(cfg.serial.sleep_t)
        ser.write(accel.to_bytes(2, byteorder='little', signed=True))

        last_pos_info = this_pos_info
        last_t = this_t
        i += 1
        # print(reward_sum)