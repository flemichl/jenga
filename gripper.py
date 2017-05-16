import RPi.GPIO as gpio
from rrb3 import *

class Gripper:

	def __init__(self):
		gpio.setmode(gpio.BCM)
		gpio.setup(13, gpio.OUT)
		self.servo = gpio.PWM(13, 333)
		self.stepper = RRB3(7, 1)

	def open(self):
		self.servo.start(20)

	def close(self):
		self.servo.start(70)

	def done(self):
		self.servo.stop()

	def downLevel(self):
		self.stepper.step_forward(0.02, 150)

	def upLevel(self):
		self.stepper.step_reverse(0.02, 150) 