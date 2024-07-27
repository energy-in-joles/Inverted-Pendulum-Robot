from serial import Serial
import time

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
    
    if len(byte_data) != 3:
        raise ValueError("Data length should be exactly 3 bytes for short.")
    pos = byte_data[0] | (byte_data[1] << 8)
    loop_i = byte_data[2]

    if pos & 0x8000:  # Check if the sign bit is set
        pos -= 0x10000  # Convert to negative value

    if loop_i & 0x80:
        loop_i -= 0x100

    return pos, loop_i

def setup_arduino(ser: Serial, max_startup_time: int = 5, auth_str: str = "<ready>") -> None:
    """
    Initializes communication with an Arduino over a serial connection.

    Args:
        ser (serial.Serial): The serial object for communication with the Arduino.
        max_startup_time (int, optional): Maximum time to wait for the Arduino to be ready, in seconds. Default is 5 seconds.
        auth_str (str, optional): The authentication string sent by the Arduino to indicate it is ready. Default is "<ready>".

    Raises:
        Exception: If the Arduino does not respond with the authentication string within the allowed startup time.
    """

    INI_BYTE_STR = b'\x00\x00'
    isReady = False
    line = ""
    start_t = time.time()
    startup_t = 0
    while not (isReady or startup_t >= max_startup_time):
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
        if line == auth_str:
            isReady = True
        startup_t = time.time() - start_t

    if not isReady:
        raise Exception("Arduino maximum setup time exceeded!")

    print("-----------------")
    print("Arduino is ready!")
    print("-----------------\n")

    # send initial accel of 0 to start the data exchange loop
    ser.write(INI_BYTE_STR)

# def calculate_velocity():
