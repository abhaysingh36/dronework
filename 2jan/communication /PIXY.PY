import time
from pymavlink import mavutil

# Replace '/dev/ttyACM0' with your own USB port
connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)

# Wait for the first heartbeat to ensure the connection is established
print("Waiting for heartbeat from Pixhawk...")
connection.wait_heartbeat()
print("Heartbeat received, connection established!")

# Function to check if the system is armed
def check_armed_status():
    heartbeat = connection.recv_match(type='HEARTBEAT', blocking=False)
    if heartbeat:
        if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("Pixhawk is ARMED")
        else:
            print("Pixhawk is DISARMED")

# Function to arm or disarm the system
def arm_disarm(action):
    if action == "ARM":
        print("Arming the system...")
        connection.mav.command_long_send(
            1,  # Target system ID (1 for Pixhawk)
            1,  # Target component ID (1 for the autopilot)
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # Command ID for arming/disarming
            1,  # Confirmation flag (1 for confirmation)
            1,  # Parameter 1: 1 to arm, 0 to disarm
            0,  # Parameter 2
            0,  # Parameter 3
            0,  # Parameter 4
            0,  # Parameter 5
            0,  # Parameter 6
            0   # Parameter 7
        )
        print("Arm command sent")
        response = connection.recv_match(type='COMMAND_ACK', blocking=True)
        print("Response:", response)
    elif action == "DISARM":
        print("Disarming the system...")
        connection.mav.command_long_send(
            1,  # Target system ID (1 for Pixhawk)
            1,  # Target component ID (1 for the autopilot)
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # Command ID for arming/disarming
            1,  # Confirmation flag (1 for confirmation)
            0,  # Parameter 1: 1 to arm, 0 to disarm
            0,  # Parameter 2
            0,  # Parameter 3
            0,  # Parameter 4
            0,  # Parameter 5
            0,  # Parameter 6
            0   # Parameter 7
        )
        print("Disarm command sent")
        response = connection.recv_match(type='COMMAND_ACK', blocking=True)
        print("Response:", response)
    else:
        print("Invalid command. Please type 'ARM' or 'DISARM'.")

# Start interactive loop
while True:
    check_armed_status()  # Check and display armed status

    # Get user input to arm or disarm
    action = input("Type 'ARM' to arm or 'DISARM' to disarm the system (or 'exit' to quit): ").strip().upper()

    if action == 'EXIT':
        print("Exiting...")
        break  # Exit the loop and stop the script
    else:
        arm_disarm(action)
    
    # Pause briefly before checking again
    time.sleep(1)
