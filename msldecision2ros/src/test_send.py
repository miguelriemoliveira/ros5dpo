#!/usr/bin/env python
import socket
import time

#UDP_IP = '172.16.33.100'   # Symbolic name meaning all available interfaces
UDP_IP = "127.0.0.1"
UDP_PORT = 5002
#MESSAGE = "v_robot, v_n_robot, omega_robot, kick_power, high_kick, low_kick, grabber_left_speed, grabber_right_speed"
MESSAGE1 = "0, 0, .1, 45, 0, 1, 50, 50"
MESSAGE2 = " --- "

print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT
print "message1:", MESSAGE1
print "message2:", MESSAGE2

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP

while 1:
    sock.sendto(MESSAGE1, (UDP_IP, UDP_PORT))
    time.sleep(.2) # delays for 5 seconds
    sock.sendto(MESSAGE2, (UDP_IP, UDP_PORT))
