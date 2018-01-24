import socket

#EyeRecTpp defaults.
IP_ADRESS = "255.255.255.255"
PORT = 2002
ADDR = (IP_ADRESS, PORT)

class EyeRecTooCapture():

    def __init__(self, log=None):
        # setting UDP socket.
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(ADDR)
        self.logger = log


    def read(self):
        data, addr = self.socket.recvfrom(8192)
        data = data
        return data.split()
