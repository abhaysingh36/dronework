import torch
import cv2

# Set device: CUDA or CPU
device = 'cuda' if torch.cuda.is_available() else 'cpu'

# Load YOLOv5 Nano model (YOLOv5n)
model = torch.hub.load('ultralytics/yolov5', 'yolov5n', device=device)  # YOLOv5 Nano model

# Initialize webcam (0 is the default webcam, change if you have multiple cameras)
cap = cv2.VideoCapture(0)

# Check if the webcam is opened correctly
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

# Start capturing the video feed
while True:
    # Read frame from webcam
    ret, frame = cap.read()

    # If the frame is not grabbed, exit
    if not ret:
        print("Error: Failed to capture image.")
        break

    # Perform inference on the current frame
    results = model(frame)
    
    # Render the results on the frame (draw bounding boxes)
    frame_with_boxes = results.render()[0]  # Results is a list, so take the first element
    
    # Display the frame with bounding boxes
    cv2.imshow('YOLOv5 Live Camera Feed', frame_with_boxes)

    # Break the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture object and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
