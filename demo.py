from time import sleep
from gripper import *
from drive import *
from localize import *
from imu import *


try:
	drive = Drive()
	grip = Gripper()
	local = Localize(20, 20, 0)
	imu = IMU()
	imu.start()
	grip.open()
	sleep(1)

	for i in xrange(3):
		drive.driveStraight(20)
		heading = imu.getHeading()
		local.updateHeading(0, int(round(heading)))
		local.updatePosition(0, 20)
		sleep(5)

	'''drive.driveStraight(-44)
	drive.turnToHeading(-90)

	grip.open()
	sleep(2)'''

finally:
	imu.stop()
	imu.join()
	grip.done()
	drive.close()
