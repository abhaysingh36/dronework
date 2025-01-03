from pymavlink import mavutil
import time

def connect_drone(connection_str):
    """
    Establish a connection to the drone.
    :param connection_str: MAVLink connection string (e.g., '/dev/ttyUSB0' or 'udp:127.0.0.1:14550').
    :return: MAVLink connection object.
    """
    print(f"Connecting to drone at {connection_str}...")
    connection = mavutil.mavlink_connection(connection_str)
    connection.wait_heartbeat()
    print("Heartbeat received! Drone is ready.")
    return connection

def arm_drone_without_motors(connection):
    """
    Arm the drone without motors connected.
    :param connection: MAVLink connection object.
    """
    print("Sending arming command (without motors)...")
    connection.mav.command_long_send(
        connection.target_system,          # Target system ID
        connection.target_component,       # Target component ID
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # Command to arm/disarm
        0,                                 # Confirmation
        1,                                 # Arm (1 to arm, 0 to disarm)
        0, 0, 0, 0, 0, 0                  # Unused parameters
    )

    print("Arming command sent. No motors are expected to respond.")
    time.sleep(3)
    print("Drone is armed for testing.")


def is_motor_armed(connection):
    """
    Check if the motors are armed.
    :param connection: MAVLink connection object.
    :return: True if motors are armed, False otherwise.
    """
    armed = connection.motors_armed()
    print(f"Motors armed status: {'Armed' if armed else 'Disarmed'}")
    return armed

def main():
    # Specify the connection string (update it based on your setup)
    connection_str = "/dev/ttyACM0"  # Example for USB connection
    connection = connect_drone(connection_str)
    arm_drone_without_motors(connection)
    is_motor_armed(connection)

if __name__ == "__main__":
    main()
