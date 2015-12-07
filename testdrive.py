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

# channels for servos (not pins--defined in arduino_comm.ino)
class servo_channels():
	GRABBER = bytes([0])
	ARM = bytes([1])

class servo_angles():
	OPEN = bytes([140])
	GRAB = bytes([95])
	LIFT = bytes([22])
	DROP = bytes([89])
	
# commands for Sabertooth 2x25
class mc_comms():
	driveForwardMotor1 = bytes([0])
	driveBackwardsMotor1 = bytes([1])
	minVoltage = bytes([2])
	maxVoltage = bytes([3])
	driveForwardMotor2 = bytes([4])
	driveBackwardsMotor2 = bytes([5])
	driveMotor1_7bit = bytes([6])
	driveMotor2_7bit = bytes([7])
	#mixed mode commands:
	driveForwardMixed = bytes([8])
	driveBackwardsMixed = bytes([9])
	driveTurnRightMixed = bytes([10])
	driveTurnLeftMixed = bytes([11])
	driveMixed_7bit = bytes([12])
	driveTurn_7bit = bytes([13])
	
	
# Set baud on Sabertooth
def mcinit(ser):
	nchout = ser.write(arduino_comms.MC_ECHO_INIT)
	nchin = ser.read(1)
	return [nchout, len(nchin)]
	
# send packetized command
def mcwrite(ser, addr, comm, data):
	nchout = ser.write(bytes.join(arduino_comms.MC_ECHO_COMM,addr,comm,data))
	nchin = ser.read(4)
	return [nchout, len(nchin)]

# set minimum battery voltage
def mcbatt(ser,addr,volts):
    data = (volts-6)*5
    return mcwrite(ser,addr,mc_comms.minVoltage,bytes([data]))
#http://www.varesano.net/blog/fabio/serial%20rs232%20connections%20python

# set servo angle
def	servoWrite(ser,channel,angle):
	return ser.write(bytes.join(arduino_comms.SERVO_ANGLE,channel,angle))
	
# attach servo
def	servoAttach(ser,channel,angle):
	return ser.write(bytes.join(arduino_comms.SERVO_ATTACH,channel))
	
# detach servo
def	servoDetach(ser,channel):
	return ser.write(bytes.join(arduino_comms.SERVO_DETACH,channel))

	

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
addr = bytes([128])		#from DIP switches on Sabertooth
speed = bytes([15])
turn = bytes([15])
stop = bytes([0])
time.sleep(2)
print(mcinit(ser))
print(servoAttach(ser,servo_channels.GRABBER))
print(servoAttach(ser,servo_channels.ARM))
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
#	h to open, j to grab, k to drop, l to lift
#	q to stop and exit
#	any other key to stop
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
	elif inchr == 'q':
		print('Stop and quit')
		print(mcwrite(ser,addr,mc_comms.driveTurnRightMixed,stop))
		print(mcwrite(ser,addr,mc_comms.driveBackwardsMixed,stop))
		print(servoDetach(ser,servo_channes.GRABBER))
		print(servoDetach(ser,servo_channes.ARM))
		break
	elif inchr == 'h':
		print('Open')
		print(setServo(ser,servo_channels.GRABBER,servo_angles.OPEN))
	elif inchr == 'j':
		print('Grab')
		print(setServo(ser,servo_channels.GRABBER,servo_angles.GRAB))
	elif inchr == 'k':
		print('Drop')
		print(setServo(ser,servo_channels.ARM,servo_angles.DROP))
	elif inchr == 'l':
		print('Lift')
		print(setServo(ser,servo_channels.ARM,servo_angles.LIFT))
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
