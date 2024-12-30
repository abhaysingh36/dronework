import sys
import torch
from PyQt6.QtWidgets import (
        QApplication, QMainWindow, QTabWidget, QWidget, QVBoxLayout,
        QLabel, QPushButton, QLineEdit, QHBoxLayout
    )
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtWebChannel import QWebChannel
from PyQt6.QtCore import (pyqtSignal, pyqtSlot, QObject, QThread, Qt)
from PyQt6.QtGui import QImage, QPixmap
import cv2
import numpy as np
from PyQt6.QtCore import QUrl
from pymavlink import mavutil
import folium
from PyQt6.QtCore import QTimer
import socket
import multiprocessing
import time 
client = None
# Function to handle communication with the server
class Client:
    def __init__(self):
        self.host='192.168.199.71' 
        self.port=5555
        # self.client=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.client.connect((self.host,self.port))        
        self.is_connected = False

    def send(self,data):
        self.client.send(data.encode("utf-8"))

    def connect(self):
       while not self.is_connected:
            try:
               self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
               self.socket.connect((self.host, self.port))
               self.is_connected = True
               print("Connected to server")
            except socket.error as e:
                print(f"Connection failed: {e}. Retrying...")
                time.sleep(5)

    def send_command(self, command):
        if self.is_connected:
            try:
                self.socket.sendall(command.encode())
            except socket.error as e:
                print(f"Error sending command: {e}")
                self.is_connected = False
                self.connect()


 

def app():


    class MarkerHandler(QObject):
        @pyqtSlot(float, float)
        def addMarker(self, lat: float, lon: float):
            print(f"Marker added at latitude: {lat}, longitude: {lon}")

    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    model = torch.hub.load('ultralytics/yolov5', 'yolov5n', device=device)
    if device == 'cuda':
        model.half() 
    class GPSCalibrationTab(QWidget):
        def __init__(self):
            super().__init__()
            layout = QVBoxLayout()

            self.map_view = QWebEngineView()
            self.web_channel = QWebChannel()
            self.marker_handler = MarkerHandler()
            self.web_channel.registerObject("markerHandler", self.marker_handler)
            self.map_view.page().setWebChannel(self.web_channel)

            self.map_view.setHtml(self.generate_google_map_html())
            layout.addWidget(self.map_view)

            coord_layout = QHBoxLayout()
            coord_layout.addWidget(QLabel("Enter GPS Coordinates:"))
            self.coordinate_input = QLineEdit()
            coord_layout.addWidget(self.coordinate_input)

            self.set_marker_button = QPushButton("Set Marker")
            self.set_marker_button.clicked.connect(self.set_marker)
            coord_layout.addWidget(self.set_marker_button)

            layout.addLayout(coord_layout)

            self.status_label = QLabel("Status: Ready")
            layout.addWidget(self.status_label)

            self.setLayout(layout)

        def generate_google_map_html(self):
            return f"""
            <!DOCTYPE html>
            <html>
            <head>
                <style>
                    html, body, #map {{
                        height: 100%;
                        margin: 0;
                        padding: 0;
                    }}
                </style>
                <script 
                    src="https://maps.googleapis.com/maps/api/js?key=AIzaSyCLGirrq1bnCqF6HBOoEJXbDS0_tX_Yjls&callback=initMap" 
                    async defer>
                </script>
                <script src="qrc:///qtwebchannel/qwebchannel.js"></script>
                <script>
                    let map;
                    let marker;

                    function initMap() {{
                        const defaultLocation = {{ lat: 51.505, lng: -0.09 }};
                        map = new google.maps.Map(document.getElementById("map"), {{
                            center: defaultLocation,
                            zoom: 13,
                        }});

                        map.addListener("click", (event) => {{
                            if (window.markerHandler) {{
                                window.markerHandler.addMarker(event.latLng.lat(), event.latLng.lng());
                            }}
                        }});
                    }}

                    function setMarker(lat, lon) {{
                        console.log('Setting marker at:', lat, lon);

                        if (marker) {{
                            marker.setMap(null);
                        }}

                        const location = {{ lat: lat, lng: lon }};
                        marker = new google.maps.Marker({{
                            position: location,
                            map: map,
                            icon: {{
                                url: "http://maps.google.com/mapfiles/ms/icons/red-dot.png"
                            }}
                        }});

                        map.panTo(location);
                        map.setZoom(15);
                    }}

                    window.onload = function() {{
                        new QWebChannel(qt.webChannelTransport, function(channel) {{
                            window.markerHandler = channel.objects.markerHandler;
                        }});
                    }};
                </script>
            </head>
            <body>
                <div id="map"></div>
            </body>
            </html>
            """ 

        def set_marker(self):
            coordinates = self.coordinate_input.text()
            try:
                lat, lon = map(float, coordinates.split(","))
                self.map_view.page().runJavaScript(f"setMarker({lat}, {lon});")
                self.status_label.setText(f"Status: Marker set at {lat}, {lon}")
            except ValueError:
                self.status_label.setText("Status: Invalid coordinates! Use 'latitude,longitude' format.")
    class MapMonitoringTab(QWidget):
        def __init__(self, drone):
            super().__init__()
            self.drone = drone

            # Layout
            self.layout = QVBoxLayout()
            self.setLayout(self.layout)

            # WebEngineView to display the map
            self.map_view = QWebEngineView()
            self.layout.addWidget(self.map_view)

            # Initialize map
            self.init_map()

            # Timer for dynamic updates
            self.update_timer = QTimer()
            self.update_timer.timeout.connect(self.update_map)
            self.update_timer.start(1000)  # Update every 1 second

        def init_map(self):
            """Initialize the map with a default location."""
            # Default location (latitude, longitude)
            self.current_location = (0.0, 0.0)
            self.map = folium.Map(location=self.current_location, zoom_start=15)

            # Save the map to an HTML file
            self.map_file = "map.html"
            self.save_map()

            # Load the map into the view
            self.map_view.setUrl(QUrl.fromLocalFile(self.map_file))

        def save_map(self):
            """Save the map to an HTML file."""
            folium.Marker(self.current_location, popup="Drone Location").add_to(self.map)
            self.map.save(self.map_file)

        def update_map(self):
            """Update the map with the drone's current GPS position."""
            try:
                if self.drone:
                    # Fetch current GPS coordinates from the drone
                    lat, lon = self.drone.get_gps_coordinates()
                    self.current_location = (lat, lon)

                    # Update the map
                    self.map = folium.Map(location=self.current_location, zoom_start=15)
                    folium.Marker(self.current_location, popup="Drone Location").add_to(self.map)

                    # Save and reload the map
                    self.save_map()
                    self.map_view.setUrl(QUrl.fromLocalFile(self.map_file))
            except Exception as e:
                print(f"Error updating map: {e}")



    class camerafeedtab(QWidget):
        def __init__(self):
            super().__init__()
            layout = QVBoxLayout()

            layout.addWidget(QLabel("Flask Camera Feed with YOLOv5", alignment=Qt.AlignmentFlag.AlignCenter))

            # QLabel to show video frames
            self.video_label = QLabel(self)
            layout.addWidget(self.video_label)

            # Create buttons to start and stop the feed
            self.start_feed_button = QPushButton("Start Feed")
            self.stop_feed_button = QPushButton("Stop Feed")
            self.stop_feed_button.setEnabled(False)

            button_layout = QHBoxLayout()
            button_layout.addWidget(self.start_feed_button)
            button_layout.addWidget(self.stop_feed_button)

            layout.addLayout(button_layout)

            self.setLayout(layout)

            # Initialize webcam feed thread
            self.video_thread = None

            # Connect buttons to methods
            self.start_feed_button.clicked.connect(self.start_feed)
            self.stop_feed_button.clicked.connect(self.stop_feed)

        def start_feed(self):
            """Start the video feed with object detection"""
            self.video_thread = ObjectDetection(self.video_label)
            self.video_thread.start()

            self.start_feed_button.setEnabled(False)
            self.stop_feed_button.setEnabled(True)

        def stop_feed(self):
            """Stop the video feed"""
            if self.video_thread is not None:
                self.video_thread.stop()
                self.video_thread.wait()  # Wait for thread to finish

            self.start_feed_button.setEnabled(True)
            self.stop_feed_button.setEnabled(False)


    class ObjectDetection(QThread):
        def __init__(self, label):
            super().__init__()
            self.label = label
            self.running = True
            self.cap = cv2.VideoCapture("http://127.0.0.1:5000/video_feed")  # Flask feed URL

            # Check if the webcam feed is opened correctly
            if not self.cap.isOpened():
                print("Error: Could not open webcam feed.")
                self.running = False

            self.frame_count = 0  # To control frame processing rate

        def run(self):
            while self.running:
                ret, frame = self.cap.read()

                # If the frame is not grabbed, break the loop
                if not ret:
                    print("Error: Failed to capture image.")
                    break

                self.frame_count += 1

                # Process every 2nd frame to reduce load
                if self.frame_count % 2 == 0:
                    # Resize the frame to a smaller resolution before inference (optional)
                    new_width = 640  # Set your desired width
                    new_height = 480  # Set your desired height
                    resized_frame = cv2.resize(frame, (new_width, new_height))

                    # Perform inference on the resized frame
                    results = model(resized_frame)

                    # Render the results on the frame (draw bounding boxes)
                    frame_with_boxes = results.render()[0]

                    # Convert frame with boxes to QImage for displaying
                    h, w, ch = frame_with_boxes.shape
                    bytes_per_line = ch * w
                    q_img = QImage(frame_with_boxes.data, w, h, bytes_per_line, QImage.Format.Format_BGR888)
                    pixmap = QPixmap(q_img)

                    # Update QLabel with the processed frame
                    self.label.setPixmap(pixmap)

                # Optional: Control the rate at which frames are processed
                # This will help ensure the thread doesn't process too many frames per second
                cv2.waitKey(1)  # 1 ms delay to let the system process other events (helpful in threading)

        def stop(self):
            """Stop the thread"""
            self.running = False
            self.cap.release()  # Release the video capture object






   
    class DroneControlWidget(QWidget):
        def __init__(self,client):
            super().__init__()
            self.drone = None
            self.client=client

            layout = QVBoxLayout()

            

            # Arm/Disarm Buttons
            arm_disarm_layout = QHBoxLayout()
            self.arm_button = QPushButton("Arm Drone")
            self.disarm_button = QPushButton("Disarm Drone")
            self.arm_button.clicked.connect(self.on_arm_clicked)
            self.disarm_button.clicked.connect(self.on_disarm_clicked)
            arm_disarm_layout.addWidget(self.arm_button)
            arm_disarm_layout.addWidget(self.disarm_button)
            layout.addLayout(arm_disarm_layout)

            # Waypoint Input
            waypoint_layout = QHBoxLayout()
            waypoint_layout.addWidget(QLabel("Waypoint (lat, lon, alt):"))
            self.waypoint_input = QLineEdit()
            waypoint_layout.addWidget(self.waypoint_input)
            self.send_waypoint_button = QPushButton("Send Waypoint")
            self.send_waypoint_button.clicked.connect(self.on_send_waypoint_clicked)
            waypoint_layout.addWidget(self.send_waypoint_button)
            layout.addLayout(waypoint_layout)

            
        
             # Connect/Disconnect Buttons
            connect_layout = QHBoxLayout()
            self.connect_button = QPushButton("Connect to Drone")
            self.disconnect_button = QPushButton("Disconnect from Drone")
            self.connect_button.clicked.connect(self.)
            self.disconnect_button.clicked.connect(self.on_disconnect_clicked)
            connect_layout.addWidget(self.connect_button)
            connect_layout.addWidget(self.disconnect_button)
            layout.addLayout(connect_layout)   

        def on_arm_clicked(self):
            """Handles the arm button click."""
            self.client.client.send("arm".encode('utf-8'))           
            

        def on_disarm_clicked(self):
            """Handles the disarm button click."""
            self.client.send("disarm".encode("utf-8"))


        def on_send_waypoint_clicked(self):

            """Handles the send waypoint button click."""
            try:
                lat, lon, alt = map(float, self.waypoint_input.text().split(","))
                self.drone.send_gps_waypoint(lat, lon, alt)
                self.status_label.setText(f"Status: Waypoint sent to {lat}, {lon}, {alt}")
            except ValueError:
                self.status_label.setText("Error: Invalid waypoint format. Use 'lat,lon,alt'")
            except Exception as e:
                self.status_label.setText(f"Error: {e}")

        def monitor_position(self):
            def update_position():
                try:
                    lat, lon, alt = self.drone.get_gps_coordinates()
                    if lat is not None:
                        self.position_label.setText(f"Position: Lat {lat}, Lon {lon}, Alt {alt}")
                    else:
                        self.position_label.setText("Error: Unable to get position")
                except Exception as e:
                    self.position_label.setText(f"Error: {e}")

            position_thread = QThread()
            position_thread.run = update_position
            position_thread.start()

    class MainWindow(QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("PyQt6 Application")

            # Initialize DroneControl instance
           
            self.tabs = QTabWidget()
            self.setCentralWidget(self.tabs)
            # self.map_monitoring_tab = MapMonitoringTab()
            self.tabs.addTab(GPSCalibrationTab(), "GPS Calibration")
            self.tabs.addTab(camerafeedtab(), "Flask Camera Feed")
            c=Client()

            self.tabs.addTab(DroneControlWidget(c), "Drone Control")



    
            self.apply_dark_theme()

        def apply_dark_theme(self):
            self.setStyleSheet("""
                QMainWindow {
                    background-color: #121212;
                    color: white;
                }
                QTabBar::tab {
                    background: #1e1e1e;
                    color: white;
                }
            """)

        def stop_all_resources(self):
           """Clean up resources like threads and connections."""
           # Stop camera feed thread
           if self.camera_feed_tab.video_thread is not None:
               self.camera_feed_tab.video_thread.stop()
               self.camera_feed_tab.video_thread.wait()
           # Disconnect drone connection
           if self.drone:
               try:
                   self.drone.connection.close()
                   print("Drone connection closed.")
               except Exception as e:
                   print(f"Error closing drone connection: {e}")
        def closeEvent(self, event):
            """Handle application exit to release resources."""
            self.stop_all_resources()
            super().closeEvent(event)









    def main():
        app = QApplication(sys.argv)
        window = MainWindow()
        window.resize(800, 600)
        window.show()
        # drone = DroneControl("/dev/ttyACM0")  # Replace with your connection string
        # drone.arm()
        # drone.set_mode("GUIDED")
        # drone.send_gps_waypoint(37.7749, -122.4194)  # Example coordinates
        # drone.monitor_position()
        sys.exit(app.exec())

    main()


if __name__== '__main__':
    queue=multiprocessing.Queue()
    apps= multiprocessing.Process(target=app)
    clients =multiprocessing.Process(target=client)
    apps.start()
    clients.start()
    apps.join()
    clients.join()
