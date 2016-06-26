# test code for controlling the 7bot robot arm
# see links below
# Getting Started
# https://www.dropbox.com/s/nhwv8j5zvaynnf8/Getting%20Started%20with%207Bot%20v1.0.pdf?dl=0
#
# Communication Instructions (ie. serial data protocol)
# https://www.dropbox.com/s/pytci253ql5lmey/Communication%20Instruction%20%28v1.0.1%29.pdf?dl=0

import serial
import time
import numpy as np

SERVO_NUM = 7 # do not change, only 7 servos on the arm
ARM_MAX_REACH = 338.25 # mm. arm will not move at all if IK commanded point is beyond this reach
ARM_MIN_REACH = 153.72 # mm. arm will not move at all if IK commanded point in XY plane is closer than this
# class defs

# robot command codes
class Flag:
    begin = 0xFE
    status = 0xF5
    speed = 0xF7
    angle = 0xF9
    IK6 = 0xFA
    IK5 = 0xFB
    IK3 = 0xFC
    read = 0xF9 # shared with the angle flag
    mask = 0x7F

# function definitions
def serialWrite(data):
    print 'Writing: 0x%02x'%(data),
    ret = ser.write(chr(data))
    print ', wrote %d bytes'%(ret)
    return ret

def setForceStatus( status ):
    "set motor force status: 0-forceless, 1-normal servo, 2-protection"
    read = serialWrite(flag.begin)
    read = serialWrite(flag.status)
    read = serialWrite(status & flag.mask)

def setSpeed(fluentEnables, speeds):
    # first process the data
    sendData = [];
    for i in range (0, SERVO_NUM):
        sendData.append( np.clip(speeds[i], 0, 250) / 10)
        if fluentEnables[i] > 0:
            sendData[i] += 64

    # send the data
    serialWrite(flag.begin)
    serialWrite(flag.speed)
    for data in sendData:
        serialWrite(data & flag.mask)


def setServoAngles( servoAngles ):
    # add the joint angle converge flag later

    # first process the data one angle at a time
    sendData = []
    for angle in servoAngles:
        sendData.append( int( angle*50/9 ) ) # conversion from example code

    # then send the data over
    # send the command bytes
    read = serialWrite(flag.begin)
    print (read)
    read = serialWrite(flag.angle)
    print (read)

    # send angles
    # 10 bits of precision, the angle needs to be broken up between bytes
    for data in sendData:
        serialWrite(( data/128) & flag.mask)
        serialWrite(data & flag.mask)

# converge flag
# determined by the last (16th) byte returned from the read command
# to come later



# begin the actual test
print "Begin barebones 7bot python test."

flag = Flag(); # create instance
start_position = np.array([0, 350, 0]) # arm nearly fully extended at z=0


# open the serial port
try:
	ser = serial.Serial(
		port='/dev/ttyACM0',
		baudrate=115200)
except:
	print 'Failed to open serial port'


if(ser.isOpen() == False):
    print "Opening serial port."
    ser.open()

# delay to let the robot boot
time.sleep(2) # wait for a few seconds

# send any necessary opening commands
setForceStatus(0) # change status to forceless
print "Setting force status to 0."
time.sleep(2)

setForceStatus(1) # change status to normal servo
print "Setting force status to 1."
time.sleep(2)
fluentEnables = [True, True, True, True, True, True, True] # set for fluent motion
speeds = [50, 50, 50, 200, 200, 200, 200] # set joint speeds
setSpeed(fluentEnables, speeds) # send the speed commands to the robot
print "Setting joint speeds."
time.sleep(1)
# put it in some reference position (eg. 90 elbow bend)
angles = [45, 115, 65, 90, 90, 90, 80]
print "Setting angles to reference position %s" % ''.join(str(e) for e in angles)
setServoAngles(angles)
print "Successfully set angles."
time.sleep(3)

# open and close the gripper
angles[-1] = 0
print "Actuating gripper."
setServoAngles(angles)
print "Actuation complete."
time.sleep(1)

angles[-1] = 90
print "Re-actuating gripper."
setServoAngles(angles)
print "Re-actuation complete."
time.sleep(1)

# set to protected mode, it should fall limp
setForceStatus(2)
print "Setting force status to 2. Watch out if the arm falls limp."
print "Test complete."

if ser.isOpen():
    print 'Closing serial port'
    ser.close()
