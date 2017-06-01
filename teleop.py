#!/usr/bin/env python

from time import sleep
from gripper import *
from drive import *
from campan import *
from turnsignal import *
from getch import *

def stepForward():
	drive.sendCommand('i')

def stepBack():
	drive.sendCommand(',')

def turnRight():
	drive.sendCommand('l')

def turnLeft():
	drive.sendCommand('j')

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

options = { 'i' : stepForward,
			',' : stepBack,
			'j' : turnLeft,
			'l' : turnRight,
			'8' : gripperDown, 
			'9' : gripperUp,
			'6' : gripperNudgeDown, 
			'7' : gripperNudgeUp,
			'2' : gripperOpen,
			'3' : gripperClose,
			's' : camDown,
			'w' : camUp,
			'd' : done }

menu = { 'i' : "Step forward",
			',' : "Step back",
			'j' : "Turn left",
			'l' : "Turn right",
			'8' : "Gripper down (level)", 
			'9' : "Gripper up (level)",
			'6' : "Gripper down (nudge)", 
			'7' : "Gripper up (nudge)",
			'2' : "Gripper open",
			'3' : "Gripper close",
			's' : "Camera tilt down",
			'w' : "Camera tilt up",
			'd' : "Send turn done signal",
			'q' : "Exit teleop" }

def printMenu():
	print "Jengrabber Teleoperation"

if __name__ == '__main__':
	try:
		playing = True
		drive = Drive()
		grip = Gripper()
		cam = CamPan()
		signal = TurnSignal()

		for key in menu:
			print key, "\t", menu[key]

		while playing:
			command = getch()
			if command in options:
				options[command]()
			
			if command == 'q':
				playing = False	

	finally:
		grip.done()
		drive.close()
		cam.done()
