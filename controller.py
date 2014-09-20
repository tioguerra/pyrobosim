from math import *
from sim import Controller
import socket
import json

class UDPCtrlServer(Controller):
    def __init__(self, sim, robot, server_port):
        self.server_port = int(server_port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('', self.server_port))
        self.socket.setblocking(0)
        super(UDPCtrlServer, self).__init__(sim, robot)
    def close(self):
        self.socket.close()
    def updatePh(self, timeStep):
        try:
            # Reads command from socket
            data, client_addr = self.socket.recvfrom(10000)
            # Adjusts servomotor setpoints accordingly
            targets = json.loads(data)
            self.robot.setTargets(targets)
            # Returns the joint angles to the client
            return_msg = json.dumps(self.robot.readSensors())
            self.socket.sendto(return_msg, client_addr)
        except socket.error:
            pass

class UDPCtrlClient:
    def __init__(self, url, server_port, client_port):
        self.server_port = int(server_port)
        self.client_port = int(client_port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('', self.client_port))
        self.server_address = (url, self.server_port)
        self.socket.setblocking(0)
    def close(self):
        self.socket.close()
    def sendCommand(self, cmd):
        # Sends servomotor setpoints to the server
        self.socket.sendto(json.dumps(cmd), self.server_address)
        try:
            # Reads joint angles from server
            data, server_ip = self.socket.recvfrom(10000)
            return json.loads(data)
        except socket.error:
            return None

class PeriodCtrl(Controller):
    def __init__(self, sim, robot):
        self.phi = 0.0
        self.phiStep = 2.0*pi / 200.0
        self.lastTime = 0
        super(PeriodCtrl, self).__init__(sim, robot)
    def updatePh(self, timeStep):
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
    def updatePh(self, timeStep):
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

