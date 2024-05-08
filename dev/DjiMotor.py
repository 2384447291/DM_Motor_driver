import numpy as np
from enum import Enum
from DjiMotorCommander import global_DjiMsgCenter
import pid
import struct

RawPos2Rad = float(0.00003994561794)  #/* 2Pi / 8191 / 3591 X 187 */
RawRpm2Rps = float(0.005453242609)    #/* 2Pi / 60 X 187 / 3591 */
PiDiv19 = float(0.1635972783)      #/* PI / 3591 X 187 */

def LoopFloatConstrain(input, minValue, maxValue):
    if (maxValue < minValue):
        return input

    if (input > maxValue):
        len = maxValue - minValue
        while input > maxValue:
            input -= len
    elif (input < minValue):
        len = maxValue - minValue
        while (input < minValue):
            input += len
    return input

class DjiMotorMode(Enum):
    RELAX_MODE = 1
    SPEED_MODE = 2
    POS_MODE = 3

class Djimotor(object):  
    def __init__(self,ID,_Motor_mode): 
        self.MotorMode = _Motor_mode
        self.id = ID
        global_DjiMsgCenter.register(self)
        self.pidSpeed = pid.Pid()
        self.pidPosition = pid.Pid()

        self.positionFdb = float(0)
        self.lastPositionFdb = float(0)
        self.speedFdb = float(0)

    
        self.positionSet = float(0)
        self.speedSet = float(0)
        self.currentSet = float(0)

        self.ecd = 0
        self.speed_rpm = 0
        self.given_current = 0
        self.rotorPosition = float(0)
        self.rotorLastPosition = float(0)
        self.feedbackpacket = np.zeros((8,1),np.uint8)  
    
    def update(self):
        if self.MotorMode == DjiMotorMode["POS_MODE"]:
            self.pidPosition.ref =  self.positionSet
            self.pidPosition.fdb = self.positionFdb
            self.pidPosition.updateresult()
            self.speedSet = self.pidPosition.result    

        self.pidSpeed.ref = self.speedSet
        self.pidSpeed.fdb = self.speedFdb
        self.pidSpeed.updateresult()
        currentSet = self.pidSpeed.result
        if self.MotorMode == DjiMotorMode["RELAX_MODE"]:
            currentSet = float(2)
        global_DjiMsgCenter.currentlist[self.id - 1] = currentSet

    def updatefeedback(self):
        self.ecd = struct.unpack("!h",self.feedbackpacket[0:2])
        self.speed_rpm = struct.unpack("!h",self.feedbackpacket[2:4])
        self.given_current = struct.unpack("!h",self.feedbackpacket[4:6])
        self.ecd = self.ecd[0]
        self.speed_rpm = self.speed_rpm[0]
        self.given_current = self.given_current[0]

        self.rotorLastPosition = self.rotorPosition
        self.lastPositionFdb = self.positionFdb
        self.rotorPosition = self.ecd * RawPos2Rad - PiDiv19#映射到-pi——pi

        self.positionFdb += LoopFloatConstrain((self.rotorPosition - self.rotorLastPosition), - PiDiv19, PiDiv19)
        self.speedFdb =  self.speed_rpm * RawRpm2Rps

    def resolve_feedback(self,packet):
        self.feedbackpacket = packet
        # Hex_Data_np = [hex(i) for i in self.feedbackpacket.reshape(8,)]
        # print(Hex_Data_np)  
        # print(self.positionFdb)
        # print(self.ecd)
        # print(self.speedFdb)
        # print(self.given_current)

