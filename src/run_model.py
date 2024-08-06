from serial import Serial
import time
import threading
from omegaconf import DictConfig
from pendulum_env import PendulumEnv
from random import uniform
import torch
import numpy as np
from stable_baselines3 import PPO, SAC

task_lock = threading.Lock()

# def train_model(cfg: DictConfig, ser: Serial, iterations=1000):
#     pendulumEnv = PendulumEnv(cfg, ser)
#     while pendulumEnv.episode_frame < iterations:
#         print(pendulumEnv.episode_frame)
#         time.sleep(cfg.serial.sleep_t)
#         # action = uniform(-2, 2)
#         action = 0
#         observation, reward, terminated, truncated, info = pendulumEnv.step(np.array([action]))
#         print(action, observation, reward, terminated, truncated, info)
#         done = terminated or truncated
#         if done:
#             pendulumEnv.reset()

def eval_model(cfg: DictConfig, ser: Serial, model_file_path: str):
    return

def train_model(cfg: DictConfig, ser: Serial, model_file_path: str):
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    env = PendulumEnv(cfg, ser)
    model = PPO("MlpPolicy", env, verbose=1, device=device)
    model.learn(total_timesteps=1000000)
    model.save(model_file_path)

    env.close()