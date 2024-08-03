from serial import Serial
import time
import threading
from omegaconf import DictConfig
from pendulum_env import PendulumEnv
from random import randint

task_lock = threading.Lock()

def train_model(cfg: DictConfig, ser: Serial, iterations=100):
    pendulumEnv = PendulumEnv(cfg, ser)
    while pendulumEnv.episode_frame < iterations:
        print(pendulumEnv.episode_frame)
        time.sleep(cfg.serial.sleep_t)
        action = randint(0, 400)
        observation, reward, terminated, truncated, info = pendulumEnv.step(action)
        print(action, observation, reward, terminated, truncated, info)
        done = terminated or truncated
        if done:
            pendulumEnv.reset()