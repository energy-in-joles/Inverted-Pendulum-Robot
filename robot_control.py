from serial import Serial
import time

def setup_arduino(ser: Serial, max_startup_time: int, auth_str: str) -> None:
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

def reset_arduino():
    return