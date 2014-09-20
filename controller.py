from math import *
from sim import Controller

class PeriodCtrl(Controller):
    def __init__(self, sim, robot):
        self.phi = 0.0
        self.phiStep = 2.0*pi / 200.0
        self.lastTime = 0
        super(PeriodCtrl,self).__init__(sim, robot)
    def updatePh(self,timeStep):
        time = int(self.sim.time*100)
        if (time != self.lastTime):
            self.robot.setLeftHipTilt(pi/6*sin(self.phi)+pi/4)
            self.robot.setLeftKnee(-pi/6*sin(self.phi)-pi/4)
            self.phi = self.phi + self.phiStep
            if self.phi > pi:
                self.phi = self.phi - 2*pi
            self.lastTime = time

class TestCtrl(Controller):
    def __init__(self, sim, robot):
        self.lastTime = 0
        super(TestCtrl,self).__init__(sim, robot)
    def updatePh(self,timeStep):
        time = int(self.sim.time*200)
        if (time % 200 == 0 and time != self.lastTime):
            if self.state == 0:
                self.state = 1
                self.robot.setLeftHipTilt(pi/6)
                self.robot.setRightHipTilt(-pi/6)
                self.robot.setLeftKnee(-pi/2)
                self.robot.setRightKnee(0)
                self.robot.setLeftAnkleTilt(-pi/4)
                self.robot.setRightAnkleTilt(0)
            elif self.state == 1:
                self.state = 2
                self.robot.setLeftHipTilt(-pi/6)
                self.robot.setRightHipTilt(pi/6)
                self.robot.setLeftKnee(0)
                self.robot.setRightKnee(-pi/2)
                self.robot.setLeftAnkleTilt(0)
                self.robot.setRightAnkleTilt(-pi/4)
            elif self.state == 2:
                self.state = 3
                self.robot.setLeftHipTilt(0)
                self.robot.setRightHipTilt(0)
                self.robot.setLeftKnee(0)
                self.robot.setRightKnee(0)
                self.robot.setLeftHipYaw(pi/4)
                self.robot.setRightHipYaw(pi/4)
                self.robot.setLeftHipRoll(pi/6)
                self.robot.setRightHipRoll(pi/6)
                self.robot.setLeftAnkleRoll(pi/6)
                self.robot.setRightAnkleRoll(pi/6)
                self.robot.setLeftAnkleTilt(0)
                self.robot.setRightAnkleTilt(0)
            elif self.state == 3:
                self.state = 0
                self.robot.setLeftHipYaw(0)
                self.robot.setRightHipYaw(0)
                self.robot.setLeftHipRoll(0)
                self.robot.setRightHipRoll(0)
                self.robot.setLeftAnkleRoll(0)
                self.robot.setRightAnkleRoll(0)
            self.lastTime = time

