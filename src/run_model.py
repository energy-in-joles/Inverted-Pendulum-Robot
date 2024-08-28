from serial import Serial
import os
from omegaconf import DictConfig
from pendulum_env import PendulumEnv
from stable_baselines3 import PPO, SAC
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.logger import configure


def eval_model(cfg: DictConfig, ser: Serial, model_file_path: str):
    env = PendulumEnv(cfg, ser)
    vec_env = DummyVecEnv([lambda: env])

    if cfg.model.name == "PPO":
        model = PPO.load(model_file_path, device="cpu")
    else:
        model = SAC.load(model_file_path, device="cpu")

    state = vec_env.reset()
    done = False
    try:
        while True:
            while not done:
                action, _states = model.predict(state)
                state, reward, done, info = vec_env.step(action)
            state = vec_env.reset()
            done = False
    except KeyboardInterrupt:
        print("Terminating program...")
    finally:
        vec_env.close()
        


def train_model(
    cfg: DictConfig,
    ser: Serial,
    current_model_file_path: str,
    log_dir: str,
    new_model_file_path: str = "",
):
    # override tensorboard logging
    class TensorboardLogging(BaseCallback):
        def __init__(self, log_dir, run_name, verbose=0):
            super(TensorboardLogging, self).__init__(verbose)
            self.log_run_path = os.path.join(log_dir, run_name)

        def _on_training_start(self) -> None:
            tensorboard_logger = configure(self.log_run_path, ["stdout", "tensorboard"])
            self.model.set_logger(tensorboard_logger)

        def _on_step(self) -> bool:
            return True

    # device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    env = PendulumEnv(cfg, ser)
    model_cfg = cfg.model
    if cfg.mode.from_scratch:
        if model_cfg.name == "PPO":
            model = PPO(
                model_cfg.policy,
                env,
                verbose=model_cfg.verbose,
                device="cpu",
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
                device="cpu",
            )
    else:
        if model_cfg.name == "PPO":
            model = PPO.load(current_model_file_path, env=env, device="cpu")
        else:
            model = SAC.load(current_model_file_path, env=env, device="cpu")

    if cfg.logging.enabled:
        print(
            f'To visualize training logs, run: tensorboard --logdir="{log_dir}", then go to http://localhost:6006/ on browser.'
        )
        callback = TensorboardLogging(log_dir=log_dir, run_name=cfg.logging.run_name)
        model.learn(total_timesteps=cfg.mode.total_timesteps, callback=callback)
    else:
        model.learn(total_timesteps=cfg.mode.total_timesteps)

    if cfg.mode.overwrite:
        model.save(current_model_file_path)
    else:
        model.save(new_model_file_path)

    env.close()
