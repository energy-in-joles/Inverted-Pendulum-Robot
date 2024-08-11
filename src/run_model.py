from serial import Serial
from omegaconf import DictConfig
from pendulum_env import PendulumEnv
from random import uniform
import torch
import numpy as np
from stable_baselines3 import PPO, SAC
import logging
from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv
from stable_baselines3.common.callbacks import BaseCallback

def eval_model(cfg: DictConfig, ser: Serial, model_file_path: str):
    if cfg.mode.model.name == "PPO":
        model = PPO.load(model_file_path)
    else:
        model = SAC.load(model_file_path)

    env = PendulumEnv(cfg, ser)
    vec_env = DummyVecEnv([lambda: env])
    obs = vec_env.reset()
    state = vec_env.reset()
    done = False

    while True:
        while not done:
            action, _states = model.predict(state)
            state, reward, done, info = vec_env.step(action)
        # if keyboard interrupt
        # env.close()

# def train_model(cfg: DictConfig, ser: Serial, model_dir_path: str, model_file_name: str):
#     iterations = 10000
#     pendulumEnv = PendulumEnv(cfg, ser)
#     print(pendulumEnv.episode_frame, iterations)
#     while pendulumEnv.episode_frame < iterations:
#         print(pendulumEnv.episode_frame)
#         time.sleep(cfg.serial.sleep_t)
#         action = uniform(-2, 2)
#         action = 0
#         observation, reward, terminated, truncated, info = pendulumEnv.step(np.array([action]))
#         print(action, observation, reward, terminated, truncated, info)
#         done = terminated or truncated
#         if done:
#             pendulumEnv.reset()

def train_model(cfg: DictConfig, ser: Serial, current_model_file_path: str, new_model_file_path: str = ""):
    # logging.basicConfig(filename='ppo_training.log', level=logging.INFO)
    # # Custom callback to log reward values
    # class CustomCallback(BaseCallback):
    #     def __init__(self, verbose=0):
    #         super(CustomCallback, self).__init__(verbose)
        
    #     def _on_step(self) -> bool:
    #         env = self.locals.get('env')
    #         if env is not None:
    #             reward = self.locals.get('rewards')
    #             obs = env.get_attr('last_pos_info')[0]
    #             logging.info(f"observation: {obs}, {reward}")
    #         return True
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    env = PendulumEnv(cfg, ser)
    model_cfg = cfg.mode.model
    if cfg.mode.from_scratch:
        if model_cfg.name == "PPO":
            model = PPO(
                model_cfg.policy, 
                env, 
                verbose=model_cfg.verbose,  
                device=device
                )
        else:
            model = SAC(
                model_cfg.policy,
                env,
                verbose=model_cfg.verbose, 
                use_sde=model_cfg.use_sde, 
                sde_sample_freq=model_cfg.sde_sample_freq, 
                use_sde_at_warmup=model_cfg.use_sde_at_warmup, 
                learning_starts=model_cfg.learning_starts, 
                device="cpu"
            )
    else:
        if model_cfg.name == "PPO":
            model = PPO.load(current_model_file_path, env=env, device=device)
        else:
            model = SAC.load(current_model_file_path, env=env, device="cpu")

    model.learn(total_timesteps=cfg.mode.total_timesteps) # , callback=CustomCallback())

    if cfg.mode.overwrite:
        model.save(current_model_file_path)
    else:
        model.save(new_model_file_path)

    env.close()
