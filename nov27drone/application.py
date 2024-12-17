import sys
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

class MarkerHandler(QObject):
    @pyqtSlot(float, float)
    def addMarker(self, lat: float, lon: float):
        print(f"Marker added at latitude: {lat}, longitude: {lon}")


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
                let marker;  // Declare the marker variable globally to manage it
        
                function initMap() {{
                    const defaultLocation = {{ lat: 51.505, lng: -0.09 }};
                    map = new google.maps.Map(document.getElementById("map"), {{
                        center: defaultLocation,
                        zoom: 13,
                    }});

                    // Enable adding marker on click
                    map.addListener("click", (event) => {{
                        if (window.markerHandler) {{
                            window.markerHandler.addMarker(event.latLng.lat(), event.latLng.lng());
                        }}
                    }});
                }}
        
                function setMarker(lat, lon) {{
                    console.log('Setting marker at:', lat, lon);  // Debugging line
        
                    // If a marker already exists, remove it before setting a new one
                    if (marker) {{
                        marker.setMap(null);
                    }}
        
                    const location = {{ lat: lat, lng: lon }};
                    marker = new google.maps.Marker({{
                        position: location,
                        map: map,
                        icon: {{
                            url: "http://maps.google.com/mapfiles/ms/icons/red-dot.png"  // Red marker icon
                        }}
                    }});
        
                    map.panTo(location);  // Move the map to the marker's position
                    map.setZoom(15);  // Optionally set a zoom level
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
            self.map_view.page().runJavaScript(f"setMarker({lat}, {lon});")  # Call JavaScript to set the marker
            self.status_label.setText(f"Status: Marker set at {lat}, {lon}")
        except ValueError:
            self.status_label.setText("Status: Invalid coordinates! Use 'latitude,longitude' format.")


class CameraFeedThread(QThread):
    frame_received = pyqtSignal(QImage)

    def __init__(self):
        super().__init__()
        self._is_running = False  # Flag to manage thread state

    def run(self):
        """Thread to capture video frames."""
        self._is_running = True
        camera = cv2.VideoCapture(0)  # Open the default camera

        if not camera.isOpened():
            print("Unable to access the camera.")
            return

        try:
            while self._is_running:
                ret, frame = camera.read()
                if not ret:
                    print("Failed to grab frame.")
                    continue

                # Convert the frame to QImage for PyQt
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Convert to RGB
                height, width, channel = frame_rgb.shape
                bytes_per_line = 3 * width
                qt_image = QImage(frame_rgb.data, width, height, bytes_per_line, QImage.Format.Format_RGB888)

                # Emit the frame to update the UI
                self.frame_received.emit(qt_image)

        finally:
            camera.release()

    def stop(self):
        """Gracefully stop the thread."""
        self._is_running = False
        self.quit()  # This will cause the thread to exit gracefully
        self.wait()  # Wait for the thread to finish


class LiveFeedTab(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout()

        layout.addWidget(QLabel("Live Feed", alignment=Qt.AlignmentFlag.AlignCenter))
        self.live_feed_label = QLabel("No feed available.")
        layout.addWidget(self.live_feed_label)

        self.start_feed_button = QPushButton("Start Feed")
        self.stop_feed_button = QPushButton("Stop Feed")
        self.stop_feed_button.setEnabled(False)

        button_layout = QHBoxLayout()
        button_layout.addWidget(self.start_feed_button)
        button_layout.addWidget(self.stop_feed_button)

        layout.addLayout(button_layout)

        self.start_feed_button.clicked.connect(self.start_feed)
        self.stop_feed_button.clicked.connect(self.stop_feed)

        self.camera_thread = CameraFeedThread()
        self.camera_thread.frame_received.connect(self.update_frame)

        self.setLayout(layout)

    def start_feed(self):
        """Start the camera feed"""
        if not self.camera_thread.isRunning():
            self.camera_thread.start()  # Start the thread that will receive and display camera frames
        self.start_feed_button.setEnabled(False)  # Disable the "Start Feed" button
        self.stop_feed_button.setEnabled(True)   # Enable the "Stop Feed" button

    def stop_feed(self):
        """Stop the camera feed without closing the app"""
        self.camera_thread.stop()  # Stop the camera feed gracefully
        self.live_feed_label.setText("No feed available.")
        self.start_feed_button.setEnabled(True)  # Re-enable the "Start Feed" button
        self.stop_feed_button.setEnabled(False)  # Disable the "Stop Feed" button

    def update_frame(self, qt_image):
        """Update the live feed with new frames"""
        self.live_feed_label.setPixmap(QPixmap.fromImage(qt_image))


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PyQt6 Application")

        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        self.tabs.addTab(GPSCalibrationTab(), "GPS Calibration")
        self.tabs.addTab(LiveFeedTab(), "Live Feed")

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


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.resize(800, 600)
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
