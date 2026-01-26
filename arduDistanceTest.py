import serial
import time

def read_distance(ser):
    while True:
        line = ser.readline().decode('utf-8').strip()
        if line:
            print(line)

def main():
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

    time.sleep(2)

    try:
        read_distance(ser)
    except KeyboardInterrupt:
        print("Program interrupted by user.")
    finally:
        print("Closing serial connection...")
        ser.close()

if __name__ == "__main__":
    main()
