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
reference_position = [0, 160, 200]
reference_vec56 = [0, 1, 0]
reference_vec67 = [1, 0, 0]
reference_end_effector = 90 # open servo gripper
upright=[0, 160, 200] # some upright position

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

        self.current_position = reference_position

    # move function
    # takes a point
    def move_accel(self, accel_x, accel_y, accel_z):


        # convert to displacements
        displacement_x = float(accel_x * X_ACCEL_TO_DISPLACEMENT)
        displacement_y = float(accel_y * Y_ACCEL_TO_DISPLACEMENT)
        displacement_z = float(accel_z * Z_ACCEL_TO_DISPLACEMENT)
        print displacement_x, displacement_y, displacement_z
        # calculate the next position
        next_position = []
        next_position.append(self.current_position[0] + displacement_x)
        next_position.append(self.current_position[1] + displacement_y)
        next_position.append(self.current_position[2] + displacement_z)

        # move the arm to the next position
        robotArm.inverseKinematics6(next_position, reference_vec56, reference_vec67, reference_end_effector)
        # update the current position
        self.current_position = next_position
        sleep(0.001)


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
