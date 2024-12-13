import cv2
import socket
import struct
import pickle

class FrameClient:
    def __init__(self, host='127.0.0.1', port=3000):
        self.host = host
        self.port = port
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((self.host, self.port))
        self.data = b""
        self.payload_size = struct.calcsize("Q")

    def get_frame(self):
        while len(self.data) < self.payload_size:
            packet = self.client_socket.recv(4096)  # Adjust buffer size as needed
            if not packet:
                return None
            self.data += packet

        # Extract message size
        packed_msg_size = self.data[:self.payload_size]
        self.data = self.data[self.payload_size:]
        msg_size = struct.unpack("Q", packed_msg_size)[0]

        # Retrieve the frame data
        while len(self.data) < msg_size:
            self.data += self.client_socket.recv(4096)

        frame_data = self.data[:msg_size]
        self.data = self.data[msg_size:]
        frame = pickle.loads(frame_data)
        return frame

    def close(self):
        self.client_socket.close()

if __name__ == "__main__":
    client = FrameClient(host='127.0.0.1', port=3000)

    try:
        while True:
            frame = client.get_frame()
            if frame is None:
                print("No frame received. Exiting.")
                break

            # Display the frame for debugging
            cv2.imshow("Received Frame", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Client exiting")
    finally:
        client.close()
        cv2.destroyAllWindows()
