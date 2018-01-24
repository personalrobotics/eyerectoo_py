import socket

#EyeRecToo defaults.
IP_ADRESS = "255.255.255.255"
PORT = 2002
ADDR = (IP_ADRESS, PORT)

# Listens for EyeRecToo data in real time and can be queried for a
# GazeDataVector at each timestep.
class LiveDataStream():

    def __init__(self, log=None):
        # setting UDP socket.
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(ADDR)
        self.logger = log

    def read(self):
        data, _ = self.socket.recvfrom(8192)
        return GazeDataVector(data)
