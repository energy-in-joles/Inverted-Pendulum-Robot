from serial import Serial
import time

def interpret_signed_short(byte_data: str) -> int:
    """
    Converts a 2-byte string into a signed 16-bit short.

    This function interprets a given 2-byte string as a signed 16-bit integer (short).
    It combines the bytes, checks the sign bit, and converts the value to a signed integer if necessary.

    Parameters:
    byte_data (str): A string of exactly 2 bytes representing the 16-bit integer.

    Returns:
    int: The signed 16-bit integer value.

    Raises:
    ValueError: If the length of byte_data is not exactly 2 bytes.
    """
    
    if len(byte_data) != 2:
        raise ValueError("Data length should be exactly 2 bytes for short.")
    value = byte_data[0] + (byte_data[1] << 8)
    # Convert to signed short
    if value & 0x8000:  # Check if the sign bit is set
        value -= 0x10000  # Convert to negative value

    return value

def setup_arduino(ser: Serial) -> None:
    """
    Initializes communication with an Arduino device over a serial connection.

    This function waits for the Arduino to send a readiness signal within a specified 
    maximum startup time. If the Arduino is ready, it sends an initial data packet to 
    start the data exchange loop. If the Arduino does not become ready within the 
    maximum startup time, an exception is raised.

    Parameters:
    ser (Serial): A serial connection object that handles communication with the Arduino.

    Raises:
    Exception: If the Arduino does not send the readiness signal within the maximum startup time (default: 5s).
    ```
    """

    MAX_STARTUP_TIME = 5
    AUTH_STR = "<ready>"
    INI_BYTE_STR = b'\x00\x00'
    isReady = False
    line = ""
    start_t = time.time()
    startup_t = 0
    while not (isReady or startup_t >= MAX_STARTUP_TIME):
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
        if line == AUTH_STR:
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
