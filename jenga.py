from serial import *
from picamera import *
from time import sleep

# all measurements in cm, and estimates for now - should measure/calibrate/confirm later
gripper_length = 10
gripper_depth = 5
robot_length = 25
block_length = 7.5
max, mid = 150, 75

turn_distance = 61 # approximate distance traveled by each wheel (opposite directions) executing a 360 degree turn

# figure out which location on robot body is "0"
home = (10, 10, 0) # set this to initial position/heading of robot on play area

# keep this set to current gripper height level (blocks on ground = 0, next layer = 1, etc.)
gripper_level = 0

# where we drop off blocks
dump = (150, 0, 0)

'''
(0, 150)					(150, 150)



	
			-----
^			jenga
|			-----
theta = 90



(0, 0)			theta = 0 ->		(150, 0)
'''

total_offset = (block_length / 2) + gripper_length + robot_length
play_poses = [(mid, mid - total_offset, 90), (mid - total_offset, mid, 0)] # add other poses?


# setup
ser = Serial('/dev/ttyACM1', 9600)
print "Starting up serial connection..."
sleep(5)
print "Done."
camera = PiCamera()


# get block choice in format: play_pose index, stepper height index
pose = 0 # hardcoded for now
level = 0 # hardcoded for now

current = home 

''' drive to tower by dead reckoning 
can have four different commands to get to different sides of tower
send combination of forward and turning commands to motor microcontroller
microcontroller will interface with encoders to close the loop
can also use IMU for correcting
'''

def turnToHeading(current, target):
	r = turn_distance * (target - current) / 360
	l = -1 * r
	command = b't ' + str(int(round(r))) + b' ' + str(int(round(l)))
	print command
	ser.write(command)
	response = ser.read()
	print response

def driveStraight(d):
	command = b't ' + str(int(round(d))) + b' ' + str(int(round(d)))
	print command
	ser.write(command)
	response = ser.read()
	print response

while True:
	''' when turn signal is on  - check for this '''	
	tx, ty, th = play_poses[pose] # t for target
	cx, cy, ch = current # c for current
	
	if (tx - cx) > (ty - cy): # this only works for lower left home position
		if ch != 0: # turn until we are facing correct direction 
			turnToHeading(ch, 0)
			ch = 0 # todo: verify this with IMU so it isn't open loop
	
		driveStraight(tx - cx)
		turnToHeading(ch, th)
		driveStraight(ty - cy)
	
	else:
		if ch != 90:
			turnToHeading(ch, 90)
			ch = 90 # same as above, verify
	
		driveStraight(ty - cy)
		turnToHeading(ch, th)
		driveStraight(tx - cx)
	
	current = (tx, ty, th) # todo: verify with IMU and loop until true
	
	''' confirm alignment by taking picture and looking for logo or something '''
	''' adjust alignment by sending turn commands to microcontroller and repeat '''
	
	# todo: use camera.capture() and save to buffer for image processing
	# todo: do Hough transform and check for straight lines
	
	# for now, just show us what camera sees for debugging purposes
	# camera.start_preview()
	# sleep(5)
	# camera.stop_preview()
	
	
	''' send commands to microcontroller to position gripper with stepper motor '''
	while gripper_level != level:
		if gripper_level > level:
			# todo: move down a level - issue stepper commands for one revolution down
			gripper_level = gripper_level - 1
	
		else:
			# todo: move up a level - issue stepper commands for one revolution up
			gripper_level = gripper_level + 1
	
	''' open/close gripper with servo '''
	# todo: set servo position to max angle to open all the way
	driveStraight(gripper_depth)
	
	servo_last, servo_current = 0, 0 # set this to current state of servo
	# keep trying to close servo incrementally until its state no longer continues to change (=> we have grasped block)
	while servo_current != servo_last:
		# todo: close servo incrementally
		servo_last = servo_current
		servo_current = 0 # todo: read from servo
		
	''' send commands to microcontroller to backup and double check IMU for slipping?'''
	
	# todo: change this to back up very slowly/incrementally to remove block
	driveStraight(-1 * (mid - total_offset))
	
	# update state (todo: actually do this with IMU)
	if current[2] == 0:
		current = (0, current[1], current[2])
	else:
		current = (current[0], 0, current[2])
	
	''' drive back to team area with dead reckoning and drop off jenga piece by opening gripper '''
	
	tx, ty, th = dump # t for target
	cx, cy, ch = current # c for current
	
	if (ty - cy) > 0: # this only works for lower left home position
		driveStraight(cy - ty) # reverse
	
	# this will be same in both cases
	turnToHeading(ch, th)
	driveStraight(tx - cx)
	
	# todo: release gripper by setting to max angle
	
	driveStraight(home[0] - tx) # reverse back to home
	current = home # todo: verify with IMU and loop until true


ser.close()
