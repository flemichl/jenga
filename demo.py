from time import sleep
from gripper import *
from drive import *


drive = Drive()
drive.driveStraight(60)

grip = Gripper()
grip.upLevel()
sleep(1)
grip.open()
sleep(1)
grip.close()
sleep(1)

grip.done()
drive.close()