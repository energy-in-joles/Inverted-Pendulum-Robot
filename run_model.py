from serial import Serial
from time import time

from util import (
    interpret_signed_short,
    setup_arduino
)

def train_model(ser: Serial):
    accel = 0
    last_t = time.time()

    setup_arduino(ser)

    while True:
        data_in = ser.read(2)
        print(data_in)
        pos = interpret_signed_short(data_in)
        delta_t = time.time() - last_t
        # t1 = threading.Thread(target=test).start()
        print(pos)
        time.sleep(0.01)
        ser.write(accel.to_bytes(2, 'little'))