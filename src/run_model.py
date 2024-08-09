from serial import Serial
import time
import threading
from omegaconf import DictConfig
from pendulum_env import PendulumEnv
from random import uniform
import torch
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
    if cfg.mode.model.name == "PPO":
        model = PPO.load(model_file_path)
    else:
        model = SAC.load(model_file_path)

    env = PendulumEnv(cfg, ser, cfg.mode.episode_length)
    state = env.reset()
    done = False

    while True:
        while not done:
            action, _states = model.predict(state)
            state, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated
        # if keyboard interrupt
        # env.close()


def train_model(cfg: DictConfig, ser: Serial, model_file_path: str):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    env = PendulumEnv(cfg, ser, cfg.mode.episode_length)
    model_cfg = cfg.mode.model
    if model_cfg.name == "PPO":
        model = PPO(
            model_cfg.policy, 
            env, 
            n_steps=model_cfg.n_steps, 
            use_sde=model_cfg.use_sde, 
            verbose=model_cfg.verbose,  
            device=device
            )
    else:
        model = SAC(
            model_cfg.policy,
            env,
            use_sde=model_cfg.use_sde, 
            verbose=model_cfg.verbose, 
            learning_starts=model_cfg.learning_starts, 
            sde_sample_freq=model_cfg.sde_sample_freq, 
            use_sde_at_warmup=model_cfg.use_sde_at_warmup, 
            train_freq=model_cfg.train_freq, 
            device=device
        )
    model.learn(total_timesteps=cfg.mode.total_timesteps)
    model.save(model_file_path)

    env.close()
