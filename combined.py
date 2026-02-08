import serial
import time
from dronekit import connect
from pymavlink import mavutil


ARDUINO_PORT = '/dev/ttyUSB0'
ARDUINO_BAUD = 9600
DRONE_PORT = '/dev/ttyACM0'
THRESHOLD = 30


def main():
    ser = None
    vehicle = None

    try:
        # Connect to Arduino
        ser = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=1)
        time.sleep(2)
        print("Connected to Arduino")

        # Connect to drone
        vehicle = connect(DRONE_PORT, wait_ready=True)
        print("Connected to drone")

        while True:
            line = ser.readline().decode('utf-8').strip()

            if not line:
                continue

            try:
                value = float(line)
                print(value)

                if value < THRESHOLD:
                    vehicle.message_factory.statustext_send(
                        mavutil.mavlink.MAV_SEVERITY_ALERT,
                        b"Distance below 30!"
                    )
                    vehicle.flush()
                    print("MAVLink alert sent")

            except ValueError:
                # In case Arduino sends non-numeric data
                print(f"Ignoring invalid data: {line}")

    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")

    finally:
        print("Closing connections...")

        if ser and ser.is_open:
            ser.close()
            print("Arduino serial closed")

        if vehicle:
            vehicle.close()
            print("Drone connection closed")


if __name__ == "__main__":
    main()
