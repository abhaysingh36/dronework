import socket

# Function to handle communication with the server
def start_client(host='192.168.199.71', port=5555):
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect((host, port))
    
    while True:
        # Get input from the user (the message to send to the server)
        message = input("Client: ")
        
        if message.lower() == 'exit':
            print("Closing connection...")
            client.close()
            break
        
        
        # Send the message to the server
        client.send(message.encode('utf-8'))
        
        # Receive the server's response
        response = client.recv(1024).decode('utf-8')
        print(f"Server: {response}")

if __name__ == '__main__':
    start_client()