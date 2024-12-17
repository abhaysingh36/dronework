import cv2
import socket
import pickle
import struct

def camera_feed_sender():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = ("127.0.0.1", 9999)

    try:
        client_socket.connect(server_address)
        print(f"Connected to server at {server_address}")
    except Exception as e:
        print(f"Failed to connect to server: {e}")
        return

    camera = cv2.VideoCapture(0)

    if not camera.isOpened():
        print("Unable to access the camera.")
        return

    try:
        while True:
            ret, frame = camera.read()
            if not ret or frame is None:
                print("Failed to grab or invalid frame.")
                continue

            # Encode the frame as JPEG
            _, buffer = cv2.imencode('.jpg', frame)
            if buffer is None:
                print("Failed to encode frame.")
                continue

            # Serialize the buffer (which is a numpy array)
            data = pickle.dumps(buffer)

            # Get data size and send it
            data_size = len(data)
            print(f"Sending frame of size {data_size} bytes.")
            client_socket.sendall(struct.pack(">L", data_size) + data)

            # Display the frame locally (optional)
            cv2.imshow("Sending Camera Feed", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("Exiting...")
                break
    except Exception as e:
        print(f"Error during streaming: {e}")
    finally:
        # Cleanup
        camera.release()
        client_socket.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    camera_feed_sender()
