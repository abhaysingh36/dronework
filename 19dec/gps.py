import time
from pymavlink import mavutil

# Connect to Pixhawk (adjust port as needed)
# Use '/dev/ttyACM0' or '/dev/ttyACM1' on Linux
# Use 'COMx' (replace x with the COM port number) on Windows
connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=9600)

def get_gps_data():
    while True:
        # Wait for a GPS message from Pixhawk
        msg = connection.recv_match(type='GPS_RAW_INT', blocking=True)
        if msg:
            # Parse the GPS data
            gps_data = {
                'time_usec': msg.time_usec,  # Timestamp (microseconds)
                'fix_type': msg.fix_type,   # GPS fix type: 1=No Fix, 2=2D Fix, 3=3D Fix
                'lat': msg.lat / 1e7,       # Latitude in degrees
                'lon': msg.lon / 1e7,       # Longitude in degrees
                'alt': msg.alt / 1e3,       # Altitude in meters
                'satellites_visible': msg.satellites_visible,  # Number of satellites
            }
            
            print("GPS Data:", gps_data)
        time.sleep(0.1)  # Add delay to avoid overloading the connection

try:
    print("Waiting for GPS data...")
    get_gps_data()
except KeyboardInterrupt:
    print("Script interrupted. Exiting.")
finally:
    connection.close()
