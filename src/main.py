import time
from serial import Serial
import torch
import os
import hydra
from omegaconf import DictConfig

from run_model import (
    train_model
)

@hydra.main(config_path="../conf", config_name="config", version_base="1.2")
def main(cfg: DictConfig):
    MODEL_FILENAME = "model_latest.pkl"
    MODEL_DIR_NAME = "model"
    SRC_PATH = os.path.dirname(__file__)
    MODEL_DIR_PATH = os.path.join(os.path.split(SRC_PATH)[0], MODEL_DIR_NAME)
    print(MODEL_DIR_PATH)

    os.makedirs(MODEL_DIR_PATH, exist_ok=True)

    if cfg.mode.name == "train":
        model_file_path = os.path.join(SRC_PATH, "train", MODEL_FILENAME)
    else:
        model_file_path = os.path.join(SRC_PATH, "eval", MODEL_FILENAME)

    try:
        model_state_dict = torch.load(model_file_path, weights_only=False)
    except:
        model_state_dict = None

    ser = Serial(port=cfg.serial.port, baudrate=cfg.serial.baudrate, timeout=cfg.serial.timeout)
    ser.reset_input_buffer()
    train_model(cfg, ser)


if __name__ == '__main__':
    main()

