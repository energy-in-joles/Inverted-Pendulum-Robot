from math import pi
from collections import namedtuple

PosInfo = namedtuple('PosInfo', ('pos', 'loop_i'))
ENCODER_STEP_PER_REV = 2400


# get angular change from last pos to this pos (in radians)
def calculate_delta_angle(last_pos_info: PosInfo, this_pos_info: PosInfo) -> float:
    delta_loop = this_pos_info.loop_i - last_pos_info.loop_i
    delta_pos = this_pos_info.pos - last_pos_info.pos # step delta, not including loop info
    delta_steps = delta_pos + ENCODER_STEP_PER_REV * delta_loop

    return delta_steps / ENCODER_STEP_PER_REV * 2 * pi

# calculate angular velocity from last pos to this pos (in radians per second)
def calculate_velocity(delta_t: float, last_pos_info: PosInfo, this_pos_info: PosInfo, modifier: float) -> float:
    delta_angle = calculate_delta_angle(last_pos_info, this_pos_info)
    return modifier * delta_angle / delta_t

# calculate angular acceleration from last vel to this vel (in radians per second square)
def calculate_acceleration(delta_t: float, last_vel: float, this_vel: float, modifier: float) -> float:
    delta_vel = this_vel - last_vel
    return modifier * delta_vel / delta_t

# get positional theta value. 0 theta at the top 
def get_theta(pos: int) -> float:
    half_rev = ENCODER_STEP_PER_REV / 2
    pos -= half_rev
    return pos * pi / half_rev

# DEBUGGING ONLY, SHOULD BE MOVED TO PendulumEnv
# reward function following pendulum environment in openai
def calculate_reward(theta: float, vel: float, accel: float, vel_weight: float, accel_weight: float) -> float:
    return -(theta ** 2 + 0.1 * vel_weight * (vel ** 2) + accel_weight * (accel ** 2))


def interpret_encoder_info(byte_data: bytes) -> tuple[int, int]:
    """
    Interprets a 3-byte sequence to extract encoder information.

    This function extracts two values from a 3-byte input:
    - A 16-bit position value, which may be negative.
    - An 8-bit loop index value, which may also be negative.

    Parameters:
    byte_data (bytes): A bytes object containing exactly 3 bytes of data.
                       The bytes should be in the order:
                       - The first byte for the low byte of the position value.
                       - The second byte for the high byte of the position value.
                       - The third byte for the loop index value.

    Returns:
    tuple[int, int]: A tuple containing two integers:
                     - The interpreted 16-bit position value, adjusted for signed representation.
                     - The interpreted 8-bit loop index value, adjusted for signed representation.

    Raises:
    ValueError: If the length of byte_data is not exactly 3 bytes.
    """
    
    if len(byte_data) != 4:
        raise ValueError("Data length should be exactly 3 bytes for short.")
    pos = byte_data[0] | (byte_data[1] >> 4) << 8
    loop_i = (byte_data[1] & 0x0F) << 7 | (byte_data[2] >> 1)
    motor_pos = byte_data[3] | ((byte_data[2] & 0x01) << 8)
    if loop_i & 0x400: # convert 11-bit number loop
        loop_i -= 0x800
    
    if motor_pos & 0x100: # convert 9-bit motor position
        motor_pos -= 0x200

    return pos, loop_i, motor_pos