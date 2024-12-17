import socket
import struct
import threading
import pickle

def handle_sender(sender_socket, receiver_socket):
    """Handles receiving camera feed from the sender and forwarding it to the receiver."""
    while True:
        try:
            # Receive the size of the incoming data
            data_size = struct.unpack(">L", sender_socket.recv(4))[0]
            data = b""
            while len(data) < data_size:
                packet = sender_socket.recv(4096)
                if not packet:
                    print("Sender disconnected.")
                    return
                data += packet

            # Forward data to the receiver
            if receiver_socket:
                receiver_socket.sendall(struct.pack(">L", len(data)) + data)
        except Exception as e:
            print(f"Error in sender thread: {e}")
            break

def start_server():
    # Setup socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(("0.0.0.0", 9999))
    server_socket.listen(5)
    print("Server is listening on port 9999...")

    sender_socket = None
    receiver_socket = None

    while True:
        # Accept connections
        client_socket, addr = server_socket.accept()
        print(f"Connection from {addr} established.")

        # If the sender or receiver is not connected, assign them
        if sender_socket is None:
            print("Sender connected.")
            sender_socket = client_socket
        elif receiver_socket is None:
            print("Receiver connected.")
            receiver_socket = client_socket
            # Start the thread for handling the sender's data
            threading.Thread(target=handle_sender, args=(sender_socket, receiver_socket), daemon=True).start()
        else:
            print("Both sender and receiver are already connected.")
            client_socket.close()

        # If either the sender or receiver disconnects, reset the connection
        if sender_socket and receiver_socket:
            try:
                # Check if the sender is still connected
                sender_socket.recv(1, socket.MSG_DONTWAIT)
            except (socket.error, BlockingIOError):
                print("Sender disconnected. Waiting for a new sender...")
                sender_socket = None
                receiver_socket = None

            try:
                # Check if the receiver is still connected
                receiver_socket.recv(1, socket.MSG_DONTWAIT)
            except (socket.error, BlockingIOError):
                print("Receiver disconnected. Waiting for a new receiver...")
                receiver_socket = None
                sender_socket = None

if __name__ == "__main__":
    start_server()
