import numpy as np
class Pid(object):  
    def __init__(self): 
        self.kp = float(0)
        self.ki = float(0)
        self.kd = float(0)
        self.maxOut = float(0)
        self.maxIOut = float(0)
        self.ref = float(0)
        self.fdb = float(0)
        self.result = float(0)
        self.pResult = float(0)
        self.iResult = float(0)
        self.dResult = float(0)
        self.err = [0.,0.,0.]
    def updateresult(self):
        self.err[2] = self.err[1]
        self.err[1] = self.err[0]
        self.err[0] = self.ref - self.fdb

        self.pResult = self.kp * self.err[0]
        self.iResult += self.ki * self.err[0]
        self.dResult = self.kd * (self.err[0] - self.err[1])
        self.iResult = np.clip(self.iResult, -self.maxIOut, self.maxIOut)
        self.result = self.pResult + self.iResult + self.dResult
        self.result = np.clip(self.result,-self.maxOut,self.maxOut)

    def clear(self):
        self.kp = float(0)
        self.ki = float(0)
        self.kd = float(0)
        self.maxOut = float(0)
        self.maxIOut = float(0)
        self.ref = float(0)
        self.fdb = float(0)
        self.result = float(0)
        self.pResult = float(0)
        self.iResult = float(0)
        self.dResult = float(0)
        self.err = [0.,0.,0.]