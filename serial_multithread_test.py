import serial
import time
import threading

lst = [0]

def test():
    print(f"running {lst[0]}")
    time.sleep(0.011)
    lst[0] += 500

if __name__ == '__main__':
    ser = serial.Serial(port='COM3', baudrate=9600, timeout=10)
    ser.reset_input_buffer()
    n = lst[0]
    while True:
        line = ser.readline().decode('utf-8').rstrip()
        t1 = threading.Thread(target=test).start()
        print(line)
        time.sleep(0.01)
        out = f"{lst[0]}\n"
        ser.write(out.encode('utf-8'))