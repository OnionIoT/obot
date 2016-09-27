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
armCommand = {
    'end_joint': reference_position,
    'velocity_x': 0,
    'velocity_y': 0,
    'velocity_z': 0,
    'vec_56': reference_vec56,
    'vec_67': reference_vec67,
    'theta3': 90,
    'theta4':90,
    'theta5':90,
    'end_effector': reference_end_effector # default closed servo gripper

    }

# create a robot arm for use by the handler
robotArm = Arm('/dev/ttyACM0', 115200)

# pass the handler an instance of the robot arm controller to work with
# the robotArm's initialization can be done beforehand
class OmegaArmHandler(SimpleHTTPRequestHandler):
    current_position = reference_position
    def __init__(self, request, client_address, server):
        SimpleHTTPRequestHandler.__init__(self, request, client_address, server)
        # move the robot to the reference position
        robotArm.inverseKinematics6(reference_position, reference_vec56, reference_vec67, reference_end_effector)

    # move function
    # takes accel values
    def move_accel(self, accel_x, accel_y, accel_z):
        # use accelerations to approximate changes in velocity, and then adjust the arm's coordinates accordingly
        # impose thresholds on the raw data
        accel_x = float(accel_x * 1) # workaround to convert unicode string to number
        accel_y = float(accel_y * 1)
        accel_z = float(accel_z * 1)

        print "accel_x:", accel_x
        print "accel_y:", accel_y
        print "accel_z:", accel_z

        deltaV_x = 0
        deltaV_y = 0
        deltaV_z = 0

        if (abs(accel_x) > accel_min):
            deltaV_x = float(accel_x * X_ACCEL_TO_DISPLACEMENT)

        if (abs(accel_y) > accel_min):
            deltaV_y = float(accel_y * Y_ACCEL_TO_DISPLACEMENT)

        if (abs(accel_z) > accel_min):
            deltaV_z = float(accel_z * Z_ACCEL_TO_DISPLACEMENT)

        armCommand['velocity_x'] += round(deltaV_x)
        armCommand['velocity_y'] += round(deltaV_y)
        armCommand['velocity_z'] += round(deltaV_z)

        # integrate the velocity to get change in displacement
        armCommand['end_joint'][0] += armCommand['velocity_x']
        armCommand['end_joint'][1] += armCommand['velocity_y']
        armCommand['end_joint'][2] += armCommand['velocity_z']

        # constrain the end_joint value to the bounds
        armCommand['end_joint'][0] = max(min(armCommand['end_joint'][0], x_max), x_min)
        armCommand['end_joint'][1] = max(min(armCommand['end_joint'][1], y_max), y_min)
        armCommand['end_joint'][2] = max(min(armCommand['end_joint'][2], z_max), z_min)

        # # convert to displacements

        # print displacement_x, displacement_y, displacement_z        # # calculate the next position
        # next_position = []
        # next_position.append(self.current_position[0] + displacement_x)
        # next_position.append(self.current_position[1] + displacement_y)
        # next_position.append(self.current_position[2] + displacement_z)

        # move the arm to the next position
        print "----------------------------"
        print "x:", armCommand['end_joint'][0]
        print "y:", armCommand['end_joint'][1]
        print "z:", armCommand['end_joint'][2]

        # robotArm.inverseKinematics6(
        #     armCommand['end_joint'],
        #     armCommand['vec_56'],
        #     armCommand['vec_67'],
        #     armCommand['end_effector'])

        # use IK3 instead here
        robotArm.inverseKinematics3(
            armCommand['end_joint'],
            armCommand['theta3'],
            armCommand['theta4'],
            armCommand['theta5'],
            armCommand['end_effector'])

        # robotArm.inverseKinematics6(next_position, reference_vec56, reference_vec67, reference_end_effector)



    def pinch(self):
        # SERVO.set_angle(5, 30)
        print "Pinch!"
        pass

    def release(self):
        # SERVO.set_angle(5, 100)
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

        self.move_accel(post_items['x'], post_items['y'], post_items['z'])

    def do_GET(self):
        if self.path == '/':
            f = open('/home/gbo/proj/onion/7bot-demo/index.html')
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(f.read())
            f.close()
            return

        elif self.path.endswith('.js'):
            f = open('/home/gbo/proj/onion/7bot-demo' + self.path)
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
