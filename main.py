import serial
import time
import threading
import numpy as np

lst = [0]

def interpret_signed_short(byte_data):
    # Ensure we have exactly 2 bytes
    if len(byte_data) != 2:
        raise ValueError("Data length should be exactly 2 bytes for short.")
    # Combine bytes to form the 16-bit integer
    value = byte_data[0] + (byte_data[1] << 8)
    # Convert to signed short
    if value & 0x8000:  # Check if the sign bit is set
        value -= 0x10000  # Convert to negative value

    return value

def test():
    # print(f"running {lst[0]}")
    time.sleep(0.011)
    lst[0] += 500

def train():
    accel = 0
    last_t = time.time()
    while True:
        #line = ser.readline().decode('utf-8').rstrip()
        data_in = ser.read(2)
        pos = interpret_signed_short(data_in)
        delta_t = time.time() - last_t
        t1 = threading.Thread(target=test).start()
        print(pos)
        time.sleep(0.01)
        ser.write(accel.to_bytes(2, 'little'))

if __name__ == '__main__':
    ser = serial.Serial(port='COM3', baudrate=9600, timeout=10)
    ser.reset_input_buffer()
    n = lst[0]
    train()

