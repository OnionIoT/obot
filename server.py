#!/usr/bin/env python

# demo controlling 7bot arm with phone motion
from arm import Arm # 7bot module
from time import sleep
import SocketServer
from OmegaArmHandler import OmegaArmHandler # creates an Arm instance in the handler script

# some constants for testing
reference_position = [-75, 150, 300]
reference_vec56 = [0, 1, 0]
reference_vec67 = [1, 0, 0]
reference_end_effector = 90 # open servo gripper
upright=[0, 160, 200] # some upright position

# main function
# most of the stuff is done in the individual files
if __name__ == '__main__':
    print "Obot start"
    print "This will take your phone's accelerometer data and turn it into robot motion!"
    PORT = 8080
    server = SocketServer.TCPServer(('', PORT), OmegaArmHandler)

    print "Serving at port", PORT, "."
    server.serve_forever()
