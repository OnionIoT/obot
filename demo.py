#!/usr/bin/env python

# demo controlling 7bot arm with phone motion
from arm import Arm # 7bot module
from time import sleep
import SocketServer
# from OmegaArmHandler import OmegaArmHandler # creates an Arm instance in the handler script

# some constants for testing
reference_position = [-75, 150, 300]
reference_vec56 = [0, 1, 0]
reference_vec67 = [1, 0, 0]
reference_end_effector = 90 # open servo gripper
upright=[0, 160, 200] # some upright position

# main function
# most of the stuff is done in the individual files
if __name__ == '__main__':
	# hard coded test values
	arm = Arm('COM6', 115200) # initialize the arm

	# test setServoAngles
	print "Controlling the robot via angles."
	servoAngles = [90, 115, 65, 90, 90, 90, 90] # robot reaches up to its left
	arm.setServoAngles(servoAngles)
	print "Angle command sent."
	sleep(2)
	
	print "Controlling the robot via angles."
	servoAngles = [90, 115, 110, 90, 90, 0, 0] # robot reaches up to its left
	arm.setServoAngles(servoAngles)
	print "Angle command sent."
	sleep(2)
	 
	print "Controlling the robot via angles."
	servoAngles = [90, 115, 40, 90, 90, 90, 90] # robot reaches up to its left
	arm.setServoAngles(servoAngles)
	print "Angle command sent."
	sleep(2)
	
	print "Controlling the robot via angles."
	servoAngles = [90, 115, 20, 90, 90, 180, 90] # robot reaches up to its left
	arm.setServoAngles(servoAngles)
	print "Angle command sent."
	sleep(2)



	# test IK6
	# print "Controlling the robot via IK6."
	# print "These commands have been verified to work in Processing."
	# print "Point 1:"
	# point = [0, 160, 200]
	# vec56 = [0, 1, 0]
	# vec67 = [1, 0, 0]
	# end_effector = 0
	# arm.inverseKinematics6(point,vec56, vec67, end_effector)
	# print "Point 1 command sent."
	# sleep(3)


	# print "Point 2:"
	# point = [50, 325, 0]
	# vec56 = [0, 1, 0]
	# vec67 = [1, 0, 0]
	# end_effector = 55
	# arm.inverseKinematics6(point,vec56, vec67, end_effector)
	# print "Point 2 command sent."
	# sleep(3)


	# print "Point 3:"
	# point = [-75, 150, 300]
	# vec56 = [0, 1, 0]
	# vec67 = [1, 0, 0]
	# end_effector = 0
	# arm.inverseKinematics6(point,vec56, vec67, end_effector)
	# print "Point 3 command sent."
	# sleep(3)


	# print "Point 4:"
	# point = [45, 200, 160]
	# vec56 = [0, 1, 0]
	# vec67 = [1, 0, 0]
	# end_effector = 55
	# arm.inverseKinematics6(point,vec56, vec67, end_effector)
	# print "Point 4 command sent."
	# sleep(3)


	# return the robot to an upright position (for repeated testing purposes)
	arm.inverseKinematics6(upright, reference_vec56, reference_vec67, reference_end_effector)
	print "Back to upright position."
	sleep(2)



	# done testing
	del arm
	print "Test complete."
