from serial import *
from time import sleep

turn_distance = 60.1

class Drive:

	def __init__(self, port='/dev/ttyACM0', baud=9600):
		try:
			self.ser = Serial(port, baud, timeout=5)
			print "Starting up serial connection..."
			sleep(5)
			print "Done."
		except SerialException:
			try:
				port = '/dev/ttyACM1'
				self.ser = Serial(port, baud)
				print "Starting up serial connection..."
				sleep(5)
				print "Done."
		
			except SerialException:
				print "No drive system connected on either port."
					



	def turnToHeading(self, current, target):
		r = turn_distance * (target - current) / 360
		l = -1 * r
		command = b't ' + str(int(round(r))) + b' ' + str(int(round(l)))
		print command
		self.ser.write(command)
		response = self.ser.read()
		print response

	def driveStraight(self, d):
		command = b't ' + str(int(round(d))) + b' ' + str(int(round(d)))
		print command
		self.ser.write(command)
		response = self.ser.read()
		print response

	def close(self):
		self.ser.close()
