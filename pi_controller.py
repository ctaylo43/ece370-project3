import socket
import sys
import threading
import time
from ctypes import *
from getkey import getkey, keys

UDP_IP = "127.0.0.1"
recPort = 5005;
sendPort = 4242;

v = 0
theta = 0
send = False

class Velocity(Structure):
	_fields_ = [("v", c_int), ("theta", c_int)]

class sendData(Structure):
	_fields_ = [("odo", c_double*3),("imu", c_double*6),("heading", c_double)]

recsock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
recsock.bind((UDP_IP, recPort))

sendsock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sendsock.bind((UDP_IP, sendPort))

print ""
print "###############################"
print ""
print "     Robot Controls"
print ""
print "UP : increase speed"
print "DOWN : decrease speed"
print "RIGHT : rotate clockwise"
print "LEFT : rotate counter-clockwise"
print "w : rotate to north"
print "a : rotate to west"
print "s : rotate to south"
print "d : rotate to east"
print "space bar : stop robot"
print "q : exit program"
print ""
print "###############################"

while True:
	input = getkey()
	
	if (input == keys.UP):
		v += 25 	# 255 / 10 = 25.5 for 10 increments to max speed
		send = True
		send_in = "UP"

	elif (input == keys.DOWN):
		v -= 25 	# 255 / 10 = 25.5 for 10 increments to max speed
		send = True
		send_in = "DOWN"

	elif (input == keys.RIGHT):
		theta -= 15
		send = True
		send_in = "RIGHT"

	elif (input == keys.LEFT):
		theta += 15
		send = True
		send_in = "LEFT"

	elif (input == 'w'):
		theta = 0
		send = True
		send_in = "w"

	elif (input == 'a'):
		theta = 270
		send = True
		send_in = "a"

	elif (input == 's'):
		theta = 180
		send = True
		send_in = "s"

	elif (input == 'd'):
		theta = 90
		send = True
		send_in = "d"

	elif (input == keys.SPACE):
		v = 0
		send = True
		send_in = "space"

	elif (input == 'q'):
		print "Exiting program"
		sys.exit()

	if v > 250: v = 250
	if v < -250: v = -250
	if theta > 360: theta = 360
	if theta < -360: theta = -360

	if (send == True):
		vel = Velocity(c_int(v),c_int(theta))
		print " "
                print "v = " + str(v)
                print "theta = " + str(theta)
                print "Sending input " + send_in
		sendsock.sendto(vel, (UDP_IP, sendPort))
