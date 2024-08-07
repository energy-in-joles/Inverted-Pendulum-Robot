from serial import Serial
import os
import hydra
from omegaconf import DictConfig

from run_model import train_model, eval_model

@hydra.main(config_path="../conf", config_name="config", version_base="1.2")
def main(cfg: DictConfig):
    MODEL_FILENAME = "model_latest.pkl"
    MODEL_DIR_NAME = "model"
    SRC_PATH = os.path.dirname(__file__)
    MODEL_DIR_PATH = os.path.join(os.path.split(SRC_PATH)[0], MODEL_DIR_NAME, cfg.mode.model.name)

    os.makedirs(MODEL_DIR_PATH, exist_ok=True)

    model_file_path = os.path.join(MODEL_DIR_PATH, MODEL_FILENAME)

    ser = Serial(port=cfg.serial.port, baudrate=cfg.serial.baudrate, timeout=cfg.serial.timeout)
    ser.reset_input_buffer()

    if cfg.mode.name == "train":
        train_model(cfg, ser, model_file_path)
    else:
        eval_model(cfg, ser, model_file_path)

if __name__ == '__main__':
    main()

