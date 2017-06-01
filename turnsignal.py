import RPi.GPIO as gpio

start_pin = 6
end_pin = 12

def timeToStart(data):
	gpio.output(end_pin, gpio.LOW)
	print "Your turn is starting!!!"

class TurnSignal:
	def __init__(self):
		gpio.setmode(gpio.BCM)
		gpio.setup(start_pin, gpio.IN)
		gpio.setup(end_pin, gpio.OUT)
		gpio.add_event_detect(start_pin, gpio.RISING, callback=timeToStart)

	def turnDone(self):
		gpio.output(end_pin, gpio.HIGH)
		
	

