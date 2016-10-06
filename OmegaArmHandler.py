#!/usr/bin/env python

# from arm import Arm
from time import sleep
from arm import Arm
import os
import sys
import json
from urlparse import urlparse, parse_qs
from SimpleHTTPServer import SimpleHTTPRequestHandler
import SocketServer
# constants
# adjust as appropriate
X_ACCEL_TO_DISPLACEMENT = 1
Y_ACCEL_TO_DISPLACEMENT = 1
Z_ACCEL_TO_DISPLACEMENT = 1
X_ACCEL_TO_VELOCITY = 5 # m/s^2 to some rough velocity command. assume they're facing the front of the robot
Y_ACCEL_TO_VELOCITY = -5 # assume they're facing the front of the robot
Z_ACCEL_TO_VELOCITY = 5
x_max = 300 # mm
x_min = -300
y_max = 300
y_min = 140
z_max = 350
z_min = 0
accel_min = 1
reference_position = [0, 160, 200]
reference_vec56 = [0, 1, 0]
reference_vec67 = [1, 0, 0]
reference_end_effector = 90 # open servo gripper
upright=[0, 160, 200] # some upright position

# global variable to track the robot's motion commands and actuator states

# create a robot arm for use by the handler
robotArm = Arm('/dev/ttyACM0', 115200)
servoAngles = [90, 115, 65, 90, 90, 90, 90]
# pass the handler an instance of the robot arm controller to work with
# the robotArm's initialization can be done beforehand
class OmegaArmHandler(SimpleHTTPRequestHandler):

	def __init__(self, request, client_address, server):
		SimpleHTTPRequestHandler.__init__(self, request, client_address, server)
		# move the robot to the reference position
		robotArm.setServoAngles(servoAngles)

	# move function
	# takes gyro values
	def set_angle(self, angle_x, angle_y, angle_z):
		angle_x = float(angle_x * 1) # workaround to convert unicode string to number
		angle_y = float(angle_y * 1)
		angle_z = float(angle_z * 1)
		if angle_x < 0:
			angle_x = 180 + angle_x
		servoAngles[0] = 180 - angle_x
		if angle_y  > 70:
			servoAngles[1] = 115
			servoAngles[2] = (180 - angle_y)
		else:	
			servoAngles[1] = (angle_y - 20)
		servoAngles[3] = 180 - angle_z
		robotArm.setServoAngles(servoAngles)
		

	def pinch(self):
		servoAngles[6] = 0
		robotArm.setServoAngles(servoAngles)
		print "Pinch!"
		pass

	def release(self):
		servoAngles[6] = 90
		robotArm.setServoAngles(servoAngles)
		print "Release!"
		pass

	def end_headers (self):
		self.send_header('Access-Control-Allow-Origin', '*')
		SimpleHTTPRequestHandler.end_headers(self)

	def do_POST(self):
		length = int(self.headers['Content-Length'])
		post_data = parse_qs(self.rfile.read(length).decode('utf-8'))
		post_items = {}
		# get the acceleration values
		for key, value in post_data.iteritems():
			post_items[key] = value[0]
		self.set_angle(post_items['x'], post_items['y'], post_items['z'])

	def do_GET(self):
		if self.path == '/':
			f = open('index.html')
			self.send_response(200)
			self.send_header('Content-type', 'text/html')
			self.end_headers()
			self.wfile.write(f.read())
			f.close()
			return

		elif self.path.endswith('.js'):
			f = open('jquery.js')
			self.send_response(200)
			self.send_header('Content-type', 'application/javascript')
			self.end_headers()
			self.wfile.write(f.read())
			f.close()
			return

		elif self.path[0:6] == '/pinch':
			self.pinch()

		elif self.path[0:8] == '/release':
			self.release()

		self.send_response(200)
		self.end_headers()

	def log_message(self, format, *args):
		return
