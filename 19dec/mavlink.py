from pymavlink import mavutil

def connect_to_pixhawk(ports, baud=115200):
    for port in ports:
        try:
            print(f"Trying to connect to Pixhawk on {port} at {baud} baud...")
            # Attempt connection to the Pixhawk
            master = mavutil.mavlink_connection(port, baud=baud)
            
            # Wait for a heartbeat to confirm connection
            print("Waiting for heartbeat...")
            master.wait_heartbeat(timeout=5)
            print(f"Connected to Pixhawk on {port}!")
            return master
        except Exception as e:
            print(f"Failed to connect on {port}: {e}")
    print("Could not connect to any Pixhawk device.")
    return None

def get_telemetry(master):
    try:
        print("Requesting GPS telemetry...")
        gps_data = master.recv_match(type='GPS_RAW_INT', blocking=True)
        if gps_data:
            print("GPS Data:")
            print(f"Latitude: {gps_data.lat / 1e7}°")
            print(f"Longitude: {gps_data.lon / 1e7}°")
            print(f"Altitude: {gps_data.alt / 1000.0}m")
        else:
            print("No GPS data received.")
    except Exception as e:
        print(f"Error getting telemetry: {e}")

if __name__ == "__main__":
    # Possible Pixhawk ports
    pixhawk_ports = ["/dev/ttyACM0", "/dev/ttyACM1"]
    
    # Try connecting to Pixhawk
    pixhawk = connect_to_pixhawk(pixhawk_ports)
    
    if pixhawk:
        # Get telemetry data
        get_telemetry(pixhawk)
