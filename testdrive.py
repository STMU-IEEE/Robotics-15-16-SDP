#Use python 3: need byte objects

import serial
import time

#http://code.activestate.com/recipes/134892-getch-like-unbuffered-character-reading-from-stdin/
from getch import getch

# Set baud on Sabertooth
def mcinit(ser):
	return ser.write(bytes([170]))

# commands for Sabertooth 2x25
class Comms():
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
	chk = (addr + comm + data) & 0b01111111
	return ser.write(bytes([addr,comm,data,chk]))

# set minimum battery voltage
def mcbatt(ser,addr,volts):
    data = (volts-6)*5
    return mcwrite(ser,addr,Comms.minVoltage,data)
#http://www.varesano.net/blog/fabio/serial%20rs232%20connections%20python

#initialize motor controller for 8N1
#issue: Sabertooth gets initialized before this program can attempt to do so.
# Is the resulting baud consistent?
# Options are either 2400, 9600, 19200, or 38400 
ser = serial.Serial('/dev/ttyACM0',38400)	#USB serial to Arduino on linux
#ser = serial.Serial('/dev/cu.usbmodem1411',9600)	#USB serial to Arduino on os x
print(ser)
# Testing: Send command to set minimum battery voltage too high (should trigger shutoff) 
#print(mcbatt(ser,addr,16))
#ser.close()
addr = 128
speed = 15
turn = 15
stop = 0
print(mcinit(ser))
#while 1:
#	speed = int(input("Forward: "))
	#time.sleep(2)
#	print(mcwrite(ser,addr,Comms.driveForwardMixed,speed))
	#print(mcwrite(ser,addr,Comms.driveBackwardsMotor2,speed))
	#print(mcwrite(ser,addr,Comms.driveForwardMotor1,speed))
#	speed = int(input("Turn right: "))
#	print(mcwrite(ser,addr,Comms.driveTurnRightMixed,speed))
	#print(mcwrite(ser,addr,Comms.driveForwardMixed,stop))
	#print(mcwrite(ser,addr,Comms.driveBackwardsMotor2,stop))
	#print(mcwrite(ser,addr,Comms.driveForwardMotor1,stop))
while 1:
	inchr = getch()
	if inchr == 'a':
		print('Left')
		mcwrite(ser,addr,Comms.driveForwardMixed,stop)
		mcwrite(ser,addr,Comms.driveTurnLeftMixed,turn)
	elif inchr == 'd':
		print('Right')
		mcwrite(ser,addr,Comms.driveForwardMixed,stop)
		mcwrite(ser,addr,Comms.driveTurnRightMixed,turn)
	elif inchr == 'w':
		print('Forward')
		mcwrite(ser,addr,Comms.driveTurnRightMixed,stop)
		mcwrite(ser,addr,Comms.driveForwardMixed,speed)
	elif inchr == 's':
		print('Backwards')
		mcwrite(ser,addr,Comms.driveTurnRightMixed,stop)
		mcwrite(ser,addr,Comms.driveBackwardsMixed,speed)
	elif inchr == '\x03': #Ctrl-C
		print('Stop and quit')
		mcwrite(ser,addr,Comms.driveTurnRightMixed,stop)
		mcwrite(ser,addr,Comms.driveBackwardsMixed,stop)		
		break
	elif inchr == 'f':
		#Don't Crash Like I Did!
		#print('Turbo forward')
		#mcwrite(ser,addr,Comms.driveTurnRightMixed,stop)
		#mcwrite(ser,addr,Comms.driveForwardMixed,127)
		#time.sleep(1)
		print('Stop')
		mcwrite(ser,addr,Comms.driveTurnRightMixed,stop)
		mcwrite(ser,addr,Comms.driveBackwardsMixed,stop)	
		break
	else:
		print('Stop')
		mcwrite(ser,addr,Comms.driveTurnRightMixed,stop)
		mcwrite(ser,addr,Comms.driveBackwardsMixed,stop)		
