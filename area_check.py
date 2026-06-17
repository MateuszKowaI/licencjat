import json
import sys
import time

from dronekit import connect, APIException
from pymavlink import mavutil

DRONE_PORT = "/dev/serial/by-id/usb-Holybro_Pixhawk6C_450046001751333337363133-if00"
DEFAULT_AREA_FILE = "area.json"
ALERT_DELAY = 2.0
NORMAL_DELAY = 0.1

def load_json(file_path):
    with open(file_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    polygon = data.get("polygon")

    if not polygon or len(polygon) < 3:
        raise ValueError("Polygon must contain at least 3 points")

    return [(float(lat), float(lon)) for lat, lon in polygon]

def connect_pixhawk():
    print("Connecting to Pixhawk...")

    while True:
        try:
            vehicle = connect(
                DRONE_PORT,
                baud=115200,
                wait_ready=True,
                heartbeat_timeout=60
            )
            print("Connected to Pixhawk")
            return vehicle

        except APIException as e:
            print(f"No heartbeat yet: {e}")
            time.sleep(3)

        except Exception as e:
            print(f"Connection failed: {e}")
            time.sleep(3)

def send_alert(vehicle, message):
    vehicle.message_factory.statustext_send(
        mavutil.mavlink.MAV_SEVERITY_ALERT,
        message.encode("utf-8")
    )
    vehicle.flush()

def get_current_position(vehicle):
    location = vehicle.location.global_frame

    if location is None or location.lat is None or location.lon is None:
        return None

    return location.lat, location.lon

def check_area(point, polygon):
    lat, lon = point
    inside = False

    n = len(polygon)
    j = n - 1

    for i in range(n):
        lat_i, lon_i = polygon[i]
        lat_j, lon_j = polygon[j]

        intersects = ((lon_i > lon) != (lon_j > lon)) and (
            lat < (lat_j - lat_i) * (lon - lon_i) / (lon_j - lon_i + 1e-12) + lat_i
        )

        if intersects:
            inside = not inside

        j = i

    return inside

def main():
    vehicle = None
    area_file = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_AREA_FILE

    try:
        polygon = load_json(area_file)
        print(f"Loaded polygon with {len(polygon)} points from {area_file}")

        vehicle = connect_pixhawk()

        send_alert(vehicle, "Companion computer connected")

        while True:
            position = get_current_position(vehicle)

            if position is None:
                continue

            inside = check_area(position, polygon)

            if not inside:
                send_alert(vehicle, "outside bounds")
                time.sleep(ALERT_DELAY)
            else:
                time.sleep(NORMAL_DELAY)

    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")

    finally:
        print("Closing connections...")

        if vehicle:
            vehicle.close()
            print("Drone connection closed")

if __name__ == "__main__":
    main()
