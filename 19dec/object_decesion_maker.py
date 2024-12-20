import cv2
import torch
from pymavlink import mavutil

# Load the YOLOv5 model
model = torch.hub.load('ultralytics/yolov5', 'yolov5n', device='cuda' if torch.cuda.is_available() else 'cpu')
if torch.cuda.is_available():
    model.half()

# Initialize connection to the drone
connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
connection.wait_heartbeat()

print("Drone connected!")

# Start video capture
cap = cv2.VideoCapture(0)  # Use the drone's camera or a local camera

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Perform object detection
    results = model(frame)
    detections = results.xyxy[0]  # [x_min, y_min, x_max, y_max, confidence, class]

    for *box, conf, cls in detections:
        # Extract bounding box and class
        x_min, y_min, x_max, y_max = map(int, box)
        detected_class = int(cls)

        # Draw bounding box
        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

        # Display detected class
        cv2.putText(frame, f"Class: {detected_class}", (x_min, y_min - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Decision-making logic
        if detected_class == 0:  # Example: Class 0 corresponds to a specific object
            print("Target detected. Moving drone towards target...")
            # Send MAVLink command to move drone
            connection.mav.mission_item_send(
                connection.target_system,
                connection.target_component,
                0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                2, 0, 0, 0, 0, 0,
                37.7749, -122.4194, 10  # Example target coordinates
            )

#     # Display video with dete
# class DroneImageControlTab(QWidget):
#     def __init__(self):
#         super().__init__()
#         layout = QVBoxLayout()

#         layout.addWidget(QLabel("Drone Control with Image Detection", alignment=Qt.AlignmentFlag.AlignCenter))
#         self.start_button = QPushButton("Start Image Detection")
#         self.stop_button = QPushButton("Stop Image Detection")
#         self.stop_button.setEnabled(False)

#         layout.addWidget(self.start_button)
#         layout.addWidget(self.stop_button)

#         self.setLayout(layout)

#         # Signals
#         self.start_button.clicked.connect(self.start_detection)
#         self.stop_button.clicked.connect(self.stop_detection)

#         self.detection_thread = None

#     def start_detection(self):
#         self.detection_thread = ImageDetectionThread()
#         self.detection_thread.start()
#         self.start_button.setEnabled(False)
#         self.stop_button.setEnabled(True)

#     def stop_detection(self):
#         if self.detection_thread:
#             self.detection_thread.stop()
#             self.detection_thread.wait()
#         self.start_button.setEnabled(True)
#         self.stop_button.setEnabled(False)


# class ImageDetectionThread(QThread):
#     def __init__(self):
#         super().__init__()
#         self.running = True

#     def run(self):
#         cap = cv2.VideoCapture(0)
#         while self.running:
#             ret, frame = cap.read()
#             if not ret:
#                 break
#             # Perform detection logic here (similar to above code)
#             # ...
#             cv2.imshow("Detection", frame)
#             if cv2.waitKey(1) & 0xFF == ord('q'):
#                 break
#         cap.release()
#         cv2.destroyAllWindows()

#     def stop(self):
#         self.running = False
