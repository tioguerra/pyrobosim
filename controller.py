from math import pi

class Controller:
    def __init__(self, sim, robot):
        self.sim = sim
        self.robot = robot
        self.state = 0
        self.sim.addController(self)
        self.lastTime = 0
    def update(self,timeStep):
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

