#Use python 3: need byte objects

import serial
import time
import struct

#getch 1.0 from pypi:
#	sudo -H pip install getch
import getch

# Commands for Arduino
class arduino_comms():
	MC_ECHO_INIT = 170
	MC_ECHO_COMM = 171
	SERVO_ATTACH = 14
	SERVO_DETACH = 15
	SERVO_WRITE	= 16
	READ_COLOR = 20
	READ_GYRO = 30

# channels for servos (not pins--defined in arduino_comm.ino)
class servo_channels():
	GRABBER = 0
	ARM = 1

class servo_angles():
	OPEN = 140
	GRAB = 72
	LIFT = 22
	DROP = 94
	
# commands for Sabertooth 2x25
class mc_comms():
	driveForwardMotor1 = 0
	driveBackwardsMotor1 = 1
	minVoltage = 2
	maxVoltage = 3
	driveForwardMotor2 = 4
	driveBackwardsMotor2 = 5
	driveMotor1_7bit = 6
	driveMotor2_7bit = 7
	#mixed mode commands:
	driveForwardMixed = 8
	driveBackwardsMixed = 9
	driveTurnRightMixed = 10
	driveTurnLeftMixed = 11
	driveMixed_7bit = 12
	driveTurn_7bit = 13
	
	
# Set baud on Sabertooth
def mcinit(ser):
	nchout = ser.write(bytes([arduino_comms.MC_ECHO_INIT]))
	nchin = ser.read(1)
	return [nchout, len(nchin)]
	
# send packetized command
def mcwrite(ser, addr, comm, data):
	nchout = ser.write(bytes([arduino_comms.MC_ECHO_COMM,addr,comm,data]))
	nchin = ser.read(4)
	return [nchout, len(nchin)]

# set minimum battery voltage
def mcbatt(ser,addr,volts):
    data = (volts-6)*5
    return mcwrite(bytes([ser,addr,mc_comms.minVoltage,data]))
#http://www.varesano.net/blog/fabio/serial%20rs232%20connections%20python

# set servo angle
def	servoWrite(ser,channel,angle):
	return ser.write(bytes([arduino_comms.SERVO_WRITE,channel,angle]))
	
# attach servo
def	servoAttach(ser,channel):
	return ser.write(bytes([arduino_comms.SERVO_ATTACH,channel]))
	
# detach servo
def	servoDetach(ser,channel):
	return ser.write(bytes([arduino_comms.SERVO_DETACH,channel]))

# read color sensor
def colorRead(ser):
	nchout = ser.write(bytes([arduino_comms.READ_COLOR]))
	#read 8 bytes as 4 big-endian unsigned short (uint16_t)
	buffer = ser.read(8)
	#result = [red, green, blue, clear]
	result = struct.unpack_from('<H',buffer)
	return [result, nchout, len(buffer)]

# read gyro sensor
def gyroRead(ser):
	nchout = ser.write(bytes([arduino_comms.READ_GYRO]))
	#read 12 bytes as 3 big-endian single-precision float
	buffer = ser.read(12)
	#result = [x, y, z]
	result = struct.unpack_from('<f',buffer)
	return [result, nchout, len(buffer)]


#initialize motor controller for 8N1
#issue: Sabertooth gets initialized before this program can attempt to do so.
# Is the resulting baud consistent?
# Options are either 2400, 9600, 19200, or 38400 
ser = serial.Serial('/dev/ttyACM0',9600)	#USB serial to Arduino on linux
#ser = serial.Serial('/dev/cu.usbmodem1421',9600)	#USB serial to Arduino on os x
print(ser)
# Testing: Send command to set minimum battery voltage too high (should trigger shutoff) 
#print(mcbatt(ser,addr,16))
#ser.close()
addr = 128		#from DIP switches on Sabertooth
speed = 15
turn = 15
stop = 0
time.sleep(2)
print('mcinit:',mcinit(ser))
print('attach grabber:',servoAttach(ser,servo_channels.GRABBER))
print('attach arm:',servoAttach(ser,servo_channels.ARM))
#while 1:
#	speed = int(input("Forward: "))
	#time.sleep(2)
#	print(mcwrite(ser,addr,mc_comms.driveForwardMixed,speed))
	#print(mcwrite(ser,addr,mc_comms.driveBackwardsMotor2,speed))
	#print(mcwrite(ser,addr,mc_comms.driveForwardMotor1,speed))
#	speed = int(input("Turn right: "))
#	print(mcwrite(ser,addr,mc_comms.driveTurnRightMixed,speed))
	#print(mcwrite(ser,addr,mc_comms.driveForwardMixed,stop))
	#print(mcwrite(ser,addr,mc_comms.driveBackwardsMotor2,stop))
	#print(mcwrite(ser,addr,mc_comms.driveForwardMotor1,stop))
	
print('Remote control:')
print('	wasd to go forward/backward, turn in place')
print('	h to open, j to grab, k to drop, l to lift')
print('	q to stop and exit')
print('	any other key to stop')
while True:
	inchr = getch.getch()
	if inchr == 'a':
		print('Left')
		print(mcwrite(ser,addr,mc_comms.driveForwardMixed,stop))
		print(mcwrite(ser,addr,mc_comms.driveTurnLeftMixed,turn))
	elif inchr == 'd':
		print('Right')
		print(mcwrite(ser,addr,mc_comms.driveForwardMixed,stop))
		print(mcwrite(ser,addr,mc_comms.driveTurnRightMixed,turn))
	elif inchr == 'w':
		print('Forward')
		print(mcwrite(ser,addr,mc_comms.driveTurnRightMixed,stop))
		print(mcwrite(ser,addr,mc_comms.driveForwardMixed,speed))
	elif inchr == 's':
		print('Backwards')
		print(mcwrite(ser,addr,mc_comms.driveTurnRightMixed,stop))
		print(mcwrite(ser,addr,mc_comms.driveBackwardsMixed,speed))
	elif inchr == 'h':
		print('Open')
		print(servoWrite(ser,servo_channels.GRABBER,servo_angles.OPEN))
	elif inchr == 'j':
		print('Grab')
		print(servoWrite(ser,servo_channels.GRABBER,servo_angles.GRAB))
	elif inchr == 'k':
		print('Drop')
		print(servoWrite(ser,servo_channels.ARM,servo_angles.DROP))
	elif inchr == 'l':
		print('Lift')
		print(servoWrite(ser,servo_channels.ARM,servo_angles.LIFT))
	elif inchr == 'g':
		print('Gyro')
		print(gyroRead(ser))
	elif inchr == 'c':
		print('Color sensor')
		print(colorRead(ser))
	elif inchr == 'q':
		print('Stop and quit')
		print(mcwrite(ser,addr,mc_comms.driveTurnRightMixed,stop))
		print(mcwrite(ser,addr,mc_comms.driveBackwardsMixed,stop))
		print(servoDetach(ser,servo_channels.GRABBER))
		print(servoDetach(ser,servo_channels.ARM))
		break
	else:
		print('Stop')
		print(mcwrite(ser,addr,mc_comms.driveTurnRightMixed,stop))
		print(mcwrite(ser,addr,mc_comms.driveBackwardsMixed,stop))	
