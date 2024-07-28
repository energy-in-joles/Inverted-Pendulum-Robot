from serial import Serial
import time

from util import PosInfo

from util import (
    interpret_encoder_info,
    calculate_velocity,
    calculate_acceleration,
    get_theta
)

from robot_control import setup_arduino

def train_model(ser: Serial, iterations=100000):
    accel = 0
    i = 0
    FIRST_TWO_FRAMES = 2
    iterations += FIRST_TWO_FRAMES # first 2 iterations no training: not enough data

    setup_arduino(ser)

    while i < iterations:
        data_in = ser.read(3)
        this_t = time.time()

        pos, loop_i = interpret_encoder_info(data_in)
        this_pos_info = PosInfo(pos, loop_i)

        # at earlier iterations, data insufficient to calculate vel/accel so we skip training
        if i > 0:
            delta_t = this_t - last_t
            this_vel = calculate_velocity(delta_t, last_pos_info, this_pos_info)
            if i > 1:
                this_accel = calculate_acceleration(delta_t, last_vel, this_vel)
                print(this_accel, this_vel)
                # t1 = threading.Thread(target=test).start()
            last_vel = this_vel
        theta = get_theta(this_pos_info.pos)
        
        time.sleep(0.01)
        ser.write(accel.to_bytes(2, 'little'))

        last_pos_info = this_pos_info
        last_t = this_t
        i += 1