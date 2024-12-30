from pymavlink import mavutil

def test_request_data_stream(connection_string, baudrate=57600):
    """
    Test the MAVLink request_data_stream_send functionality.
    """
    try:
        # Initialize connection to the drone
        connection = mavutil.mavlink_connection(connection_string, baud=baudrate)
        print("Waiting for heartbeat...")
        
        # Wait for heartbeat
        connection.wait_heartbeat(timeout=10)
        print(f"Heartbeat received from system {connection.target_system}, component {connection.target_component}")
        
        # Request all data streams at 1 Hz
        print("Requesting all data streams at 1 Hz...")
        connection.mav.request_data_stream_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            1,  # Rate in Hz
            1   # Enable stream
        )
        
        print("Data stream request sent. Monitoring for messages...")
        
        # Monitor for messages
        while True:
            msg = connection.recv_match(blocking=True)
            if not msg or msg.get_type() == 'BAD_DATA':
                continue
            print(f"Received message: {msg}")

    except KeyboardInterrupt:
        print("Exiting...")
    except Exception as e:
        print(f"Error: {e}")

# Define your connection string (adjust '/dev/ttyACM0' or baudrate if necessary)
connection_string = "/dev/ttyACM0"
test_request_data_stream(connection_string)
