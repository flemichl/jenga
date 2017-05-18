from mpu6050 import mpu6050
from threading import Thread, Event
from time import sleep

class IMU(Thread):
	def __init__(self):
		Thread.__init__(self)
		self.done = Event()
		self.heading = 0
		self.x, self.y = 0, 0
		self.vx, self.vy = 0, 0
		self.accel = {'x' : [0] * 10, 'y' : [0] * 10, 'z' : [0] * 10}
		self.imu = mpu6050(0x68)
		self.resetGyro()
		self.resetAccel()

	def resetGyro(self):
		self.gyro_baseline = {'x' : 0, 'y' : 0, 'z' : 0}
		# sample gyro to get baseline readings
		for i in xrange(1000):
			data = self.imu.get_gyro_data()
			self.gyro_baseline['x'] += data['x']
			self.gyro_baseline['y'] += data['y']
			self.gyro_baseline['z'] += data['z']
		self.gyro_baseline['x'] /= 1000
		self.gyro_baseline['y'] /= 1000
		self.gyro_baseline['z'] /= 1000
		#print self.gyro_baseline

	def resetAccel(self):
		self.accel_baseline = {'x' : 0, 'y' : 0, 'z' : 0}
		# sample gyro to get baseline readings
		for i in xrange(1000):
			data = self.imu.get_accel_data()
			self.accel_baseline['x'] += data['x']
			self.accel_baseline['y'] += data['y']
			self.accel_baseline['z'] += data['z']
		self.accel_baseline['x'] /= 1000
		self.accel_baseline['y'] /= 1000
		self.accel_baseline['z'] /= 1000
		#print self.accel_baseline

	def getHeading(self):
		return self.heading

	def getPosition(self):
		return self.vx, self.vy, self.x, self.y

	def stop(self):
		self.done.set()

	def stopped(self):
		return self.done.is_set()

	def run(self):
		dt = 0.01
		i = 0
		while not self.done.is_set():
			i += 1

			# integrate gyro over time
			data = self.imu.get_gyro_data()
			z = data['z'] - self.gyro_baseline['z']
			self.heading += dt * z

			# integrate acceleration over time
			data = self.imu.get_accel_data()
			self.accel['x'][i % 10] = data['x']
			self.accel['y'][i % 10] = data['y']
			self.accel['z'][i % 10] = data['z']
		
			ax = (sum(self.accel['x']) / 10) - self.accel_baseline['x']
			if abs(ax) > 0.1:
				self.vx += dt * ax
				self.x += (dt * self.vx) + (0.5 * dt * dt * ax)
			ay = (sum(self.accel['y']) / 10) - self.accel_baseline['y']
			if abs(ay) > 0.1:
				self.vy += dt * ay
				self.y += dt * self.vy + (0.5 * dt * dt * ay)

			if i % 1000 == 0:
				print ax
				print ay
				print self.accel

			sleep(dt)
			
