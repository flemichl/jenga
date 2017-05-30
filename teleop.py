#!/usr/bin/env python

from time import sleep
from gripper import *
from drive import *
from campan import *
from turnsignal import *

def stepForward():
	drive.driveStraight(30)

def stepBack():
	drive.driveStraight(-30)

def nudgeForward():
	drive.driveStraight(10)

def nudgeBack():
	drive.driveStraight(-10)

def turnRight():
	drive.turnToHeading(-45)

def turnLeft():
	drive.turnToHeading(45)

def gripperUp():
	grip.upLevel()

def gripperDown():
	grip.downLevel()

def gripperOpen():
	grip.open()

def gripperClose():
	grip.close()

def camUp():
	cam.up()

def camDown():
	cam.down()

def done():
	print "Ending turn."
	signal.turnDone()

options = { 'i' : stepForward,
			',' : stepBack,
			'u' : nudgeForward,
			'm' : nudgeBack,
			'j' : turnLeft,
			'l' : turnRight,
			'8' : gripperDown, 
			'9' : gripperUp,
			'2' : gripperOpen,
			'3' : gripperClose,
			'a' : camDown,
			'q' : camUp,
			'd' : done }

def printMenu():
	print "Jengrabber Teleoperation"

if __name__ == '__main__':
	try:
		drive = Drive()
		grip = Gripper()
		cam = CamPan()
		signal = TurnSignal()

		while True:
			command = raw_input()

			if len(command) > 0 and command[0] in options:
				options[command[0]]()


	finally:
		grip.done()
		drive.close()
		cam.done()
