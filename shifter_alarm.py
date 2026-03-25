import time
from dronekit import connect
from pymavlink import mavutil

from gpiozero import Device, DistanceSensor
from gpiozero.pins.lgpio import LGPIOFactory

Device.pin_factory = LGPIOFactory()

DRONE_PORT = "/dev/serial/by-id/usb-Holybro_Pixhawk6C_450046001751333337363133-if00"
THRESHOLD = 30   #w centymetrach
GAP = 0.05       #50ms
MAX_DIST = 5     #w metrach

sensor1 = DistanceSensor(echo=17, trigger=23, max_distance=MAX_DIST)
sensor2 = DistanceSensor(echo=27, trigger=24, max_distance=MAX_DIST)
sensor3 = DistanceSensor(echo=22, trigger=25, max_distance=MAX_DIST)


def connect_pixhawk():
    print("Waiting for Pixhawk heartbeat...")

    while True:
        try:
            master = mavutil.mavlink_connection(
                DRONE_PORT,
                baud=115200
            )

            master.wait_heartbeat(timeout=60)
            print("Pixhawk heartbeat received")

            vehicle = connect(
                DRONE_PORT,
                baud=115200,
                wait_ready=False
            )

            print("Connected to Pixhawk")
            return vehicle

        except Exception as e:
            print(f"Pixhawk not ready: {e}")
            time.sleep(3)


def check_distance(vehicle, distance, sensor_id):
    print(f"Sensor {sensor_id}: {distance:.2f} cm")

    if distance < THRESHOLD:
        message = f"Distance below {THRESHOLD}"

        vehicle.message_factory.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_ALERT,
            message.encode("utf-8")
        )
        vehicle.flush()

        print("MAVLink alert sent")


def main():
    vehicle = None

    try:
        vehicle = connect_pixhawk()

        vehicle.message_factory.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_ALERT,
            b"Companion computer connected"
        )
        vehicle.flush()

        while True:
            d1 = sensor1.distance * 100 * MAX_DIST
            check_distance(vehicle, d1, 1)
            time.sleep(GAP)

            d2 = sensor2.distance * 100 * MAX_DIST
            check_distance(vehicle, d2, 2)
            time.sleep(GAP)

            d3 = sensor3.distance * 100 * MAX_DIST
            check_distance(vehicle, d3, 3)
            time.sleep(GAP)

            print("---------------------------")

    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")

    finally:
        print("Closing connections...")

        if vehicle:
            vehicle.close()
            print("Drone connection closed")


if __name__ == "__main__":
    main()
