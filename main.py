import serial
if __name__ == '__main__':
    ser = serial.Serial(port='COM3', baudrate=9600, timeout=.1)
    ser.reset_input_buffer()
    n = 0
    while True:
        n += 1
        out = f"{n}\n"
        ser.write(out.encode('utf-8'))
        line = ser.readline().decode('utf-8').rstrip()
        print(line)


     #while True:
     #   if ser.in_waiting > 0:
     #       line = ser.readline().decode('utf-8').rstrip()
     #       print(line)