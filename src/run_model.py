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

def train_model(cfg: DictConfig, ser: Serial):

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    env = PendulumEnv(cfg, ser)
    model = PPO("MlpPolicy", env, verbose=1, device=device)
    model.learn(total_timesteps=10000000)

    # vec_env = model.get_env()
    # obs = vec_env.reset()
    # for i in range(1000):
    #     action, _states = model.predict(obs, deterministic=True)
    #     obs, reward, done, info = vec_env.step(action)

    env.close()