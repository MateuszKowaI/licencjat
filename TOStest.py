from dronekit import connect
from pymavlink import mavutil

vehicle = connect('/dev/ttyACM0', wait_ready=True)

vehicle.message_factory.statustext_send(
    mavutil.mavlink.MAV_SEVERITY_WARNING,
    b"test message"
)
