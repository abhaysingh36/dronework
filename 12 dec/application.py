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
from PyQt6.QtCore import QUrl
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


class FlaskFeedTab(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout()

        layout.addWidget(QLabel("Flask Camera Feed", alignment=Qt.AlignmentFlag.AlignCenter))
        self.web_view = QWebEngineView()
        layout.addWidget(self.web_view)

        # Use QUrl to set the URL for the video feed from the Flask server
        self.web_view.setUrl(QUrl("http://127.0.0.1:5000/video_feed"))

        # Create buttons to start and stop the feed
        self.start_feed_button = QPushButton("Start Feed")
        self.stop_feed_button = QPushButton("Stop Feed")
        self.stop_feed_button.setEnabled(False)

        button_layout = QHBoxLayout()
        button_layout.addWidget(self.start_feed_button)
        button_layout.addWidget(self.stop_feed_button)

        layout.addLayout(button_layout)

        # Connect buttons to methods
        self.start_feed_button.clicked.connect(self.start_feed)
        self.stop_feed_button.clicked.connect(self.stop_feed)

        self.setLayout(layout)

    def start_feed(self):
        """Start the video feed"""
        self.web_view.setUrl(QUrl("http://127.0.0.1:5000/video_feed"))
        self.start_feed_button.setEnabled(False)
        self.stop_feed_button.setEnabled(True)

    def stop_feed(self):
        """Stop the video feed"""
        self.web_view.setUrl(QUrl())  # Clear the URL to stop the feed
        self.start_feed_button.setEnabled(True)
        self.stop_feed_button.setEnabled(False)






class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PyQt6 Application")

        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        self.tabs.addTab(GPSCalibrationTab(), "GPS Calibration")
      
        self.tabs.addTab(FlaskFeedTab(), "Flask Camera Feed")

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
