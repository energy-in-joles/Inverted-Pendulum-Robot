import time
import threading
from serial import Serial
import torch
import argparse

from run_model import (
    train_model
)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="choose parameters for DQN model.")
    parser.add_argument('-m', '--mode', type=str, required=False, help='mode: train or eval (default: eval)')
    MODEL_FILENAME = "model_latest.pkl"
    MODEL_PATH = "."

    args = parser.parse_args()
    if args.mode == "train":
        path = f"{MODEL_PATH}/train/{MODEL_FILENAME}"
    else:
        path = f"{MODEL_PATH}/eval/{MODEL_FILENAME}"

    try:
        model_state_dict = torch.load(f"{MODEL_PATH}/train/{MODEL_FILENAME}", weights_only=False)
    except:
        model_state_dict = None


              
    ser = Serial(port='COM3', baudrate=9600, timeout=10)
    ser.reset_input_buffer()
    train_model(ser)

