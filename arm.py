# demo using 7bot at Hamilton Maker Faire
import serial
import warnings
import time



class Arm:
    def __init__(self, port_name, baud_rate):
        try:
            self.port = serial.Serial(port_name, baud_rate, timeout=0.1);
        except x:
            warnings.warn("can't open the serial port to control the robot arm");
            raise x;
        time.sleep(2) # wait a few seconds for the robot to boot

        # load flags and constants
        self.flag = self.Flag()
        self.SERVO_NUM = 7 # do not change, only 7 servos on the arm
        self.ARM_MAX_REACH = 338.25 # mm. arm will not move if IK commanded point is beyond this reach
        self.ARM_MIN_REACH = 153.72 # mm. limit is actually a bit larger depending on Z height

        # default arm speeds and settings
        self.setForceStatus(1)
        time.sleep(1)
        speeds = [50, 50, 50, 200, 200, 200, 200]
        fluentEnables = [True, True, True, True, True, True, True]
        self.setSpeed(fluentEnables, speeds)
        time.sleep(2)

    def __del__(self):
        self.port.close()

    class Flag: # command bytes to be written
        def __init__(self):
            self.begin = 0xFE
            self.status = 0xF5
            self.speed = 0xF7
            self.angle = 0xF9
            self.IK6 = 0xFA
            self.IK5 = 0xFB
            self.IK3 = 0xFC
            self.read = 0xF9
            self.mask = 0x7F

    def clip(self, x, lo, hi): # can't get numpy to install atm
        return lo if x <= lo else hi if x >= hi else x

    # write commands to the robot's serial port
    def robotWriteSerial(self,data):
        #print 'Writing: 0x%02x'%(data),
        ret = self.port.write(chr(data))
        self.port.flush()
        #print ', wrote %d bytes'%(ret)
        #print "Reading input: ", self.port.read()
        return ret

    def setForceStatus(self,status):
        "set motor force status: 0-forceless, 1-normal servo, 2-protection"
        read = self.robotWriteSerial(self.flag.begin)
        read = self.robotWriteSerial(self.flag.status)
        read = self.robotWriteSerial(status & self.flag.mask)

    # set motor speeds
    def setSpeed(self, fluentEnables, speeds):
        # first process the data
        sendData = [];
        for i in range (0, self.SERVO_NUM):
            sendData.append( self.clip(speeds[i], 0, 250) / 10)
            if fluentEnables[i] > 0:
                sendData[i] += 64

        # send the data
        self.robotWriteSerial(self.flag.begin)
        self.robotWriteSerial(self.flag.speed)
        for data in sendData:
            self.robotWriteSerial(data & self.flag.mask)

    def setServoAngles(self, servoAngles):
        # add the joint angle converge flag later

        # first process the data one angle at a time
        sendData = []
        for angle in servoAngles:
            sendData.append( int( angle*50/9 ) ) # conversion from example code

        # then send the data over
        # send the command bytes
        read = self.robotWriteSerial(self.flag.begin)
        print (read)
        read = self.robotWriteSerial(self.flag.angle)
        print (read)

        # send angles
        # 10 bits of precision, the angle needs to be broken up between bytes
        for data in sendData:
            self.robotWriteSerial((data / 128) & self.flag.mask)
            self.robotWriteSerial(data & self.flag.mask)

    def inverseKinematics6(self, joint6, vec56, vec67, end_effector_signal):
        # TODO: add convergence

        # process data
        # constrain the joint vector to the bounds given in the processing example
        joint6_constrained = [ self.clip(joint6[0], -500, 500), self.clip(joint6[1], -500, 500), self.clip(joint6[2], -500, 500) ]
        sendData = [] # prepare the commands to send

        # prepare joint 6 vector data
        sendData.append(int(abs(joint6_constrained[0])))
        if joint6_constrained[0] < 0:
            sendData[0] += 1024
        sendData.append(int(abs(joint6_constrained[1])))
        if joint6_constrained[1] < 0:
            sendData[-1] += 1024
        sendData.append(int(abs(joint6_constrained[2])))
        if joint6_constrained[2] < 0:
            sendData[-1] += 1024

        # prepare vector 56 vector data
        sendData.append(int(abs(vec56[0])))
        if vec56[0] < 0:
            sendData[-1] += 1024
        sendData.append(int(abs(vec56[1])))
        if vec56[1] < 0:
            sendData[-1] += 1024
        sendData.append(int(abs(vec56[2])))
        if vec56[2] < 0:
            sendData[-1] += 1024

        # prepare vector 67 vector data
        sendData.append(int(abs(vec67[0])))
        if vec67[0] < 0:
            sendData[-1] += 1024
        sendData.append(int(abs(vec67[1])))
        if vec67[1] < 0:
            sendData[-1] += 1024
        sendData.append(int(abs(vec67[2])))
        if vec67[2] < 0:
            sendData[-1] += 1024

        # prepare end effector signal data
        sendData.append(int(end_effector_signal * 50 / 9))

        # now send the data to the robot

        self.robotWriteSerial(self.flag.begin)
        self.robotWriteSerial(self.flag.IK6)
        for data in sendData:
            self.robotWriteSerial((data / 128) & self.flag.mask)
            self.robotWriteSerial(data & self.flag.mask)
