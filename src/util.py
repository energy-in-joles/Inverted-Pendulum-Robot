from math import pi
from collections import namedtuple

PosInfo = namedtuple("PosInfo", ("pos", "loop_i", "motor_pos"))
ENCODER_STEP_PER_REV = 2400


# calculate angular velocity from last pos to this pos (in radians per second)
def calculate_velocity(
    delta_t: float, last_pos_info: PosInfo, this_pos_info: PosInfo, modifier: float
) -> float:
    delta_angle = _calculate_delta_angle(last_pos_info, this_pos_info)
    return modifier * delta_angle / delta_t


# calculate angular acceleration from last vel to this vel (in radians per second square)
def calculate_acceleration(
    delta_t: float, last_vel: float, this_vel: float, modifier: float
) -> float:
    delta_vel = this_vel - last_vel
    return modifier * delta_vel / delta_t


# get positional theta value. 0 theta at the top
def get_theta(pos: int) -> float:
    half_rev = ENCODER_STEP_PER_REV / 2
    pos -= half_rev
    return pos * pi / half_rev


def calculate_norm_motor_velocity(
    delta_t: float,
    last_norm_motor_vel: float,
    this_norm_motor_vel: float,
    modifier: float,
) -> float:
    return modifier * (this_norm_motor_vel - last_norm_motor_vel) / delta_t


# normalise motor position to be within -2 and 2
def normalise_motor_pos(
    motor_pos: int, motor_half_range: int, motor_norm_half_range: int
) -> float:
    return motor_pos / motor_half_range * motor_norm_half_range


# interpret output buffer from robot:
# encoder position arranged in little endian (23 bits)
# motor position arranged in big endian (9 bits)
# convert to signed value
def interpret_encoder_info(byte_data: bytes) -> tuple[int, int]:
    if len(byte_data) != 4:
        raise ValueError("Data length should be exactly 3 bytes for short.")
    encoder_pos = byte_data[0] | byte_data[1] << 8 | ((byte_data[2] & 0xFC) << 14)
    motor_pos = byte_data[3] | ((byte_data[2] & 0x03) << 8)

    if encoder_pos & 0x200000:
        encoder_pos -= 0x400000
    if motor_pos & 0x200:  # convert 9-bit motor position
        motor_pos -= 0x400

    pos, loop_i = _process_raw_encoder_pos(encoder_pos)

    return pos, loop_i, motor_pos


# split raw encoder position into a wrapped pos value (0 to 2400) and loop_i representing the loop_index (number of full revolutions from center)
def _process_raw_encoder_pos(encoder_pos: int):
    # Normalize position to be within range [0, ENCODER_STEP_PER_REV)
    pos = encoder_pos % ENCODER_STEP_PER_REV
    if pos < 0:
        pos += ENCODER_STEP_PER_REV

    # Calculate loop index
    loop_i = encoder_pos // ENCODER_STEP_PER_REV
    if pos < 0:
        loop_i -= 1
    return pos, loop_i


# get angular change from last pos to this pos (in radians)
def _calculate_delta_angle(last_pos_info: PosInfo, this_pos_info: PosInfo) -> float:
    delta_loop = this_pos_info.loop_i - last_pos_info.loop_i
    delta_pos = (
        this_pos_info.pos - last_pos_info.pos
    )  # step delta, not including loop info
    delta_steps = delta_pos + ENCODER_STEP_PER_REV * delta_loop

    return delta_steps / ENCODER_STEP_PER_REV * 2 * pi
