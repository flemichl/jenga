import RPi.GPIO as gpio
from rrb3 import *
from time import sleep

class Gripper:
	open_dc = 15
	close_dc = 50

	def __init__(self):
		gpio.setmode(gpio.BCM)
		gpio.setup(13, gpio.OUT)
		self.servo = gpio.PWM(13, 333)
		self.stepper = RRB3(7, 6)
		self.servo.start(self.open_dc)
		self.current = self.open_dc

	def open(self):
		while self.current > self.open_dc:
			self.current = max(self.current - 5, self.open_dc)
			self.servo.start(self.current)
			sleep(0.2)

	def close(self):
		while self.current < self.close_dc:
			self.current = min(self.current + 5, self.close_dc)
			self.servo.start(self.current)
			sleep(0.2)
		
	def done(self):
		self.servo.stop()

	def downLevel(self):
		self.stepper.step_forward(0.02, 150)

	def upLevel(self):
		self.stepper.step_reverse(0.02, 150) 
