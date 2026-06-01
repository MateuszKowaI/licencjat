import time
import math
import csv
from datetime import datetime

from dronekit import connect, APIException
from pymavlink import mavutil

from gpiozero import Device, DistanceSensor
from gpiozero.pins.lgpio import LGPIOFactory

Device.pin_factory = LGPIOFactory()

DRONE_PORT = "/dev/serial/by-id/usb-Holybro_Pixhawk6C_450046001751333337363133-if00"

REACTION_TIME_S = 1.6
GAP = 0.05                  # 50 ms
MAX_DIST_M = 4.5            # w metrach
MAX_DIST_CM = MAX_DIST_M * 100.0
LOG_FILE = "sensor_threshold_log.csv"

SENSOR_ANGLES_DEG = {
    1: -45.0,
    2: 0.0,
    3: 45.0
}

sensor1 = DistanceSensor(echo=17, trigger=23, max_distance=MAX_DIST_M)
sensor2 = DistanceSensor(echo=27, trigger=24, max_distance=MAX_DIST_M)
sensor3 = DistanceSensor(echo=22, trigger=25, max_distance=MAX_DIST_M)


def init_log_file():
    try:
        with open(LOG_FILE, "x", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "timestamp",
                "sensor_id",
                "distance_cm",
                "toward_speed_m_s",
                "threshold_cm"
            ])
    except FileExistsError:
        pass


def log_sensor_data(sensor_id, distance_cm, toward_speed, threshold_cm):
    with open(LOG_FILE, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            datetime.now().isoformat(timespec="seconds"),
            sensor_id,
            f"{distance_cm:.2f}",
            f"{toward_speed:.2f}",
            f"{threshold_cm:.2f}"
        ])


def connect_pixhawk():
    print("łączenie z Pixhawk...")

    while True:
        try:
            vehicle = connect(
                DRONE_PORT,
                baud=115200,
                wait_ready=True,
                heartbeat_timeout=60
            )
            print("połączono z Pixhawk")
            return vehicle

        except APIException as e:
            print(f"brak heartbeat: {e}")
            time.sleep(3)

        except Exception as e:
            print(f"nie połączono: {e}")
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


def calculate_threshold_cm(toward_speed):
    closing_speed = max(0.0, toward_speed)

    threshold_cm = closing_speed * REACTION_TIME_S * 100.0

    threshold_cm = min(threshold_cm, MAX_DIST_CM)

    return threshold_cm


def send_alert(vehicle, sensor_id):
    message = f"obstacle {sensor_id}"

    vehicle.message_factory.statustext_send(
        mavutil.mavlink.MAV_SEVERITY_ALERT,
        message.encode("utf-8")[:50]
    )
    vehicle.flush()


def check_distance(vehicle, distance_cm, sensor_id, v_forward, v_right):
    sensor_angle = SENSOR_ANGLES_DEG[sensor_id]
    toward_speed = get_speed_toward_sensor(v_forward, v_right, sensor_angle)
    threshold_cm = calculate_threshold_cm(toward_speed)

    log_sensor_data(sensor_id, distance_cm, toward_speed, threshold_cm)

    print(
        f"Sensor {sensor_id}: "
        f"dystans={distance_cm:.2f} cm, "
        f"prędkość={toward_speed:.2f} m/s, "
        f"próg={threshold_cm:.2f} cm"
    )

    if threshold_cm > 0.0 and distance_cm < threshold_cm:
        send_alert(vehicle, sensor_id, distance_cm, threshold_cm)


def main():
    vehicle = None

    try:
        init_log_file()
        vehicle = connect_pixhawk()

        vehicle.message_factory.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_ALERT,
            b"Companion computer connected"
        )
        vehicle.flush()

        while True:
            v_forward, v_right = get_body_velocity(vehicle)

            d1 = sensor1.distance * 100.0
            check_distance(vehicle, d1, 1, v_forward, v_right)
            time.sleep(GAP)

            d2 = sensor2.distance * 100.0
            check_distance(vehicle, d2, 2, v_forward, v_right)
            time.sleep(GAP)

            d3 = sensor3.distance * 100.0
            check_distance(vehicle, d3, 3, v_forward, v_right)
            time.sleep(GAP)

    except KeyboardInterrupt:
        pass

    finally:
        if vehicle:
            vehicle.close()


if __name__ == "__main__":
    main()
