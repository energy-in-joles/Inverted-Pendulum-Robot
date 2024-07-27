from serial import Serial
import time

from util import (
    interpret_encoder_info,
    setup_arduino
)

def train_model(ser: Serial):
    accel = 0
    last_t = time.time()

    setup_arduino(ser)

    while True:
        data_in = ser.read(3)
        pos, loop_i = interpret_encoder_info(data_in)
        delta_t = time.time() - last_t
        # t1 = threading.Thread(target=test).start()
        print(pos, loop_i)
        time.sleep(0.01)
        ser.write(accel.to_bytes(2, 'little'))