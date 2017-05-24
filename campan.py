import RPi.GPIO as gpio
from time import sleep

class CamPan:
	low_dc = 30
	high_dc = 50

	def __init__(self):
		gpio.setmode(gpio.BCM)
		gpio.setup(16, gpio.OUT)
		self.servo = gpio.PWM(16, 333)
		self.current = (self.low_dc + self.high_dc) / 2
		self.servo.start(self.current) 

	def down(self):
		if self.current < self.high_dc:
			self.current = min(self.current + 2, self.high_dc)
			self.servo.start(self.current)

	def up(self):
		if self.current > self.low_dc:
			self.current = max(self.current - 2, self.low_dc)
			self.servo.start(self.current)
		
	def done(self):
		self.servo.stop()
	

