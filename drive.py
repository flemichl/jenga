from serial import *
from time import sleep

turn_distance = 60.1
k = 1

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
					
	
	'''def turnToHeading(self,  angle):
		l = turn_distance * angle / 360
		r = -1 * l
		command = b't ' + str(int(round(r*k))) + b' ' + str(int(round(l*k)))
		print command
		self.ser.write(command)
		print self.ser.read()

	def driveStraight(self, d):
		command = b't ' + str(int(round(d*k))) + b' ' + str(int(round(d*k)))
		print command
		self.ser.write(command)
		print self.ser.read()'''

	def sendCommand(self, command):
		self.ser.write(command)
		
	def close(self):
		self.ser.close()
