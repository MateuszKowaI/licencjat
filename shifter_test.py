from gpiozero import DistanceSensor
from time import sleep

# GPIO pins (BCM numbering)
TRIG_PIN = 4
ECHO_PIN = 17

# Create the ultrasonic sensor object
sensor = DistanceSensor(
    echo=ECHO_PIN,
    trigger=TRIG_PIN,
    max_distance=4,   # maximum distance in meters
    threshold_distance=0.3
)

print("Ultrasonic distance measurement started. Press Ctrl+C to stop.")

try:
    while True:
        # gpiozero returns distance as a fraction of max_distance
        distance_cm = sensor.distance * 100

        print(f"Distance: {distance_cm:.2f} cm")

        sleep(0.5)

except KeyboardInterrupt:
    print("\nProgram stopped.")
