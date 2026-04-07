import time
import math
from dronekit import connect
from pymavlink import mavutil

from gpiozero import Device, DistanceSensor
from gpiozero.pins.lgpio import LGPIOFactory

Device.pin_factory = LGPIOFactory()

DRONE_PORT = "/dev/serial/by-id/usb-Holybro_Pixhawk6C_450046001751333337363133-if00"

MIN_DISTANCE_CM = 30
MIN_SPEED = 0.3 # w m/s
GAP = 0.05 # 50ms
MAX_DIST = 8

SENSOR_ANGLES_DEG = {
    1: -45.0,
    2: 0.0,
    3: 45.0
}

sensor1 = DistanceSensor(echo=17, trigger=23, max_distance=MAX_DIST)
sensor2 = DistanceSensor(echo=27, trigger=24, max_distance=MAX_DIST)
sensor3 = DistanceSensor(echo=22, trigger=25, max_distance=MAX_DIST)


def connect_pixhawk():
    while True:
        try:
            conn = mavutil.mavlink_connection(
                DRONE_PORT,
                baud=115200
            )

            conn.wait_heartbeat(timeout=60)
            conn.close()

            vehicle = connect(
                DRONE_PORT,
                baud=115200,
                wait_ready=True
            )

            return vehicle

        except Exception:
            time.sleep(3)


def get_body_velocity(vehicle):
    velocity = vehicle.velocity
    yaw = vehicle.attitude.yaw

    if velocity is None or yaw is None:
        return 0.0, 0.0

    vn, ve, _ = velocity

    v_forward = vn * math.cos(yaw) + ve * math.sin(yaw)
    v_right = -vn * math.sin(yaw) + ve * math.cos(yaw)

    return v_forward, v_right


def get_speed_toward_sensor(v_forward, v_right, sensor_angle_deg):
    angle_rad = math.radians(sensor_angle_deg)
    return v_forward * math.cos(angle_rad) + v_right * math.sin(angle_rad)


def send_alert(vehicle):
    message = "obstacle"

    vehicle.message_factory.statustext_send(
        mavutil.mavlink.MAV_SEVERITY_ALERT,
        message.encode("utf-8")
    )
    vehicle.flush()


def check_distance(vehicle, distance_cm, sensor_id, v_forward, v_right):
    sensor_angle = SENSOR_ANGLES_DEG[sensor_id]
    toward_speed = get_speed_toward_sensor(v_forward, v_right, sensor_angle)

    if distance_cm < MIN_DISTANCE_CM and toward_speed > MIN_SPEED:
        send_alert(vehicle)


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
            v_forward, v_right = get_body_velocity(vehicle)

            d1 = sensor1.distance * 100 * MAX_DIST
            check_distance(vehicle, d1, 1, v_forward, v_right)
            time.sleep(GAP)

            d2 = sensor2.distance * 100 * MAX_DIST
            check_distance(vehicle, d2, 2, v_forward, v_right)
            time.sleep(GAP)

            d3 = sensor3.distance * 100 * MAX_DIST
            check_distance(vehicle, d3, 3, v_forward, v_right)
            time.sleep(GAP)

    except KeyboardInterrupt:
        pass

    finally:
        if vehicle:
            vehicle.close()


if __name__ == "__main__":
    main()
