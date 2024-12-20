#!/usr/bin/env python3

from pymavlink import mavutil
import time

# Connect to Pixhawk (already done in your script)
connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

# Wait for heartbeat to confirm connection
print("Waiting for heartbeat...")
connection.wait_heartbeat()
print("Connected to Pixhawk!")

# Function to handle incoming messages
def default_message_sent_by_pixhawk(msg):
    print(f"Received message: {msg.get_type()}")
    print(msg.to_dict())


# Listen for messages and handle them
try:
    print("Listening for MAVLink messages...")
    while True:
        msg = connection.recv_match()
        if msg:
            default_message_sent_by_pixhawk(msg)
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Exiting...")
