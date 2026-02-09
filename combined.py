import serial
import time
from dronekit import connect
from pymavlink import mavutil

ARDUINO_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
ARDUINO_BAUD = 9600
DRONE_PORT = "/dev/serial/by-id/usb-Holybro_Pixhawk6C_450046001751333337363133-if00"
THRESHOLD = 30


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


def main():
	ser = None
	vehicle = None

	try:
		# Connect to Arduino
		ser = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=1)
		time.sleep(2)
		print("Connected to Arduino")

		# Connect to Pixhawk
		vehicle = connect_pixhawk()

		vehicle.message_factory.statustext_send(
			mavutil.mavlink.MAV_SEVERITY_ALERT,
			b"Companion computer connected"
		)
		vehicle.flush()

		while True:
			line = ser.readline().decode("utf-8").strip()

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
