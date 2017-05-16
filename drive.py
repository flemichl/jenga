from serial import *
from time import sleep
from mpu6050 import mpu6050
from numpy import arange

turn_distance = 60.1

class Drive:

	def __init__(self, port='/dev/ttyACM0', baud=9600):
		self.heading = 0
		self.imu = mpu6050(0x68)
		self.gyro_baseline = {'x' : 0, 'y' : 0, 'z' : 0}
		
		# sample gyro to get baseline readings
		for i in xrange(10):
			data = self.imu.get_gyro_data()
			self.gyro_baseline['x'] += data['x']
			self.gyro_baseline['y'] += data['y']
			self.gyro_baseline['z'] += data['z']
		self.gyro_baseline['x'] /= 10
		self.gyro_baseline['y'] /= 10
		self.gyro_baseline['z'] /= 10
		print self.gyro_baseline
		
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
					
	
	def estimateHeading(self):
		dt = 0.1
		for i in range(0, int(round(t / dt))):
			data = self.imu.get_gyro_data()
			dz = data['z'] - self.gyro_baseline['z']
			self.heading += dt * dz
			sleep(dt)
		print self.heading


	def turnToHeading(self,  target):
		while abs(target - self.heading) > 5:
			l = turn_distance * (target - self.heading) / 360
			r = -1 * l
			command = b't ' + str(int(round(r))) + b' ' + str(int(round(l)))
			print command
			self.ser.write(command)
		self.estimateHeading()

	def driveStraight(self, d):
		command = b't ' + str(int(round(d))) + b' ' + str(int(round(d)))
		print command
		self.ser.write(command)
		self.estimateHeading()

	def close(self):
		self.ser.close()
