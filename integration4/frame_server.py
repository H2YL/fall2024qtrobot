import cv2
import socket
import struct
import pickle
import threading

class FrameServer:
    def __init__(self, host='0.0.0.0', port=3000, camera_index=6):
        self.host = host
        self.port = port
        self.cap = cv2.VideoCapture(camera_index)

        if not self.cap.isOpened():
            raise Exception("Cannot open camera")

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)  # Allow up to 5 clients

    def start(self):
        print(f"Frame server started at {self.host}:{self.port}")
        while True:
            client_socket, client_address = self.server_socket.accept()
            print(f"Connection from {client_address}")
            # Start a new thread for each client
            threading.Thread(target=self._send_frames, args=(client_socket,)).start()

    def _send_frames(self, client_socket):
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    break

                # Serialize frame
                data = pickle.dumps(frame)
                # Send message length and frame
                client_socket.sendall(struct.pack("Q", len(data)) + data)

        except Exception as e:
            print(f"Connection error: {e}")
        finally:
            client_socket.close()

    def stop(self):
        self.cap.release()
        self.server_socket.close()

if __name__ == "__main__":
    server = FrameServer(host='127.0.0.1', port=3000, camera_index=6)
    try:
        server.start()
    except KeyboardInterrupt:
        print("Shutting down server")
        server.stop()
