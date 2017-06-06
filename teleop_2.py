#!/usr/bin/env python

from time import sleep
from gripper import *
from drive import *
from campan import *
from turnsignal import *
from getch import *
import pdb


def stepForward():
	drive.sendCommand('i')

def stepBack():
	drive.sendCommand(',')

def turnRight():
	drive.sendCommand('l')

def turnLeft():
	drive.sendCommand('j')

def stop():
	drive.sendCommand('k')

# the next four are all autonomous commands
def driveDistForward(d):
	print("Driving Forward: %s cm" %d)
	drive.sendCommand('f ' + str(abs(d)) )

def driveDistBackward(d):
	print("Driving Backward: %s cm" %d)
	drive.sendCommand('b ' + str(abs(d)) )

def driveTurnLeft(r):
	print("Turning Left: %s radians" %r)
	drive.sendCommand('L ' + str(r))
	
def driveTurnRight(r):
	print("Turning Right: %s radians" %r)
	drive.sendCommand('R ' + str(abs(r)))

def gripperUp():
	grip.upLevel()
	print "Done."

def gripperDown():
	grip.downLevel()
	print "Done."

def gripperNudgeUp():
	grip.upNudge()
	print "Done."

def gripperNudgeDown():
	grip.downNudge()
	print "Done."

def gripperOpen():
	grip.open()
	print "Done."

def gripperClose():
	grip.close()
	print "Done"

def camUp():
	cam.up()

def camDown():
	cam.down()

def done():
	print "Ending turn."
	signal.turnDone()

def printMenu():
	print "Jengrabber Teleoperation"
	for key in menu:
			print key, "\t", menu[key]
	print("\n")

options = { 'i' : stepForward,
			',' : stepBack,
			'j' : turnLeft,
			'l' : turnRight,
			'k' : stop,
			'8' : gripperDown, 
			'9' : gripperUp,
			'6' : gripperNudgeDown, 
			'7' : gripperNudgeUp,
			'2' : gripperOpen,
			'3' : gripperClose,
			's' : camDown,
			'w' : camUp,
			'd' : done,
			'p' : printMenu }

menu = { 'i' : "Step forward",
			',' : "Step back",
			'j' : "Turn left",
			'l' : "Turn right",
			'k' : "Stop",
			'8' : "Gripper down (level)", 
			'9' : "Gripper up (level)",
			'6' : "Gripper down (nudge)", 
			'7' : "Gripper up (nudge)",
			'2' : "Gripper open",
			'3' : "Gripper close",
			's' : "Camera tilt down",
			'w' : "Camera tilt up",
			'p' : "Print Menu",
			'd' : "Send turn done signal",
			'q' : "Exit teleop" }



if __name__ == '__main__':
	try:
		playing = True
		drive = Drive()
		grip = Gripper()
		cam = CamPan()
		signal = TurnSignal()

		printMenu()

		while playing:
			command = getch()
			if command in options:
				options[command]()
			elif command in ['f','b','L','R']:
				if command in ['f','b']:
					c = raw_input("Enter Distance")
					if command == 'f':
						driveDistForward(abs(int(c)))
					elif command == 'b':
						driveDistBackward(abs(int(c)))
				elif command in ['L','R']:
					c = raw_input("Enter Rotation")
					if command == 'L':
						driveTurnLeft(abs(float(c)))
					elif command == 'R':
						driveTurnRight(abs(float(c)))
				else:
					print("Not sure how i got here")

			
			if command == 'q':
				playing = False
				stop()

	finally:
		grip.done()
		drive.close()
		cam.done()
