#Use python 3: need byte objects

import serial
import time

#getch 1.0 from pypi:
#	sudo -H pip install getch
import getch

# Commands for Arduino
class arduino_comms():
	MC_ECHO_INIT = bytes([170])
	MC_ECHO_COMM = bytes([171])
	SERVO_ATTACH = bytes([14])
	SERVO_DETACH = bytes([15])
	SERVO_ANGLE	= bytes([16])

# Set baud on Sabertooth
def mcinit(ser):
	nchout = ser.write(arduino_comms.MC_ECHO_INIT)
	nchin = ser.read(1)
	return [nchout, len(nchin)]

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
	
# send packetized command
def mcwrite(ser, addr, comm, data):
	nchout = ser.write(arduino_comms.MC_ECHO_COMM)
	nchout += ser.write(bytes([addr,comm,data]))
	nchin = ser.read(4)
	return [nchout, len(nchin)]

# set minimum battery voltage
def mcbatt(ser,addr,volts):
    data = (volts-6)*5
    return mcwrite(ser,addr,mc_comms.minVoltage,data)
#http://www.varesano.net/blog/fabio/serial%20rs232%20connections%20python

# set servo angle
def	setServo(ser,channel,angle):
	nchout = ser.write(arduino_comms.
	return nchout

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
print(mcinit(ser))
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
	
# Remote control:
#	wasd to go forward/backward, turn in place
#	q to stop and exit
#	any other key to stop
while 1:
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
	elif inchr == 'q':
		print('Stop and quit')
		print(mcwrite(ser,addr,mc_comms.driveTurnRightMixed,stop))
		print(mcwrite(ser,addr,mc_comms.driveBackwardsMixed,stop))
		break
	elif inchr == 'h':
		print('Open')
		print(setServo
	#elif inchr == 'f':
		#Don't Crash Like I Did!
		#print('Turbo forward')
		#mcwrite(ser,addr,mc_comms.driveTurnRightMixed,stop)
		#mcwrite(ser,addr,mc_comms.driveForwardMixed,127)
		#time.sleep(1)
		#print('Stop')
		#mcwrite(ser,addr,mc_comms.driveTurnRightMixed,stop)
		#mcwrite(ser,addr,mc_comms.driveBackwardsMixed,stop)	
		break
	else:
		print('Stop')
		print(mcwrite(ser,addr,mc_comms.driveTurnRightMixed,stop))
		print(mcwrite(ser,addr,mc_comms.driveBackwardsMixed,stop))	
