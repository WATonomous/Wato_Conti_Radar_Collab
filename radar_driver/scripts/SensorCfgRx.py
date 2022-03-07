#Send Radar a SensorCfg Packet

import socket
import struct

UDP_IP = "192.168.1.2"
UDP_PORT = 31123

radomeDamping = 100#???
farCenterFreq = 0#???
nearCenterFreq = 0#???

#Need to convert ints to byte strings
#Doc: https://docs.python.org/3/library/struct.html#byte-order-size-and-alignment
message = struct.pack('!I', radomeDamping)
message += struct.pack('!B', farCenterFreq)
message += struct.pack('!B', nearCenterFreq)

print("{}\n{}\n".format(message, len(message)))

sock = socket.socket(socket.AF_INET, # Internet
             socket.SOCK_DGRAM) # UDP
sock.sendto(message, (UDP_IP, UDP_PORT))