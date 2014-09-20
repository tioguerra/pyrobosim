import ode
import vtk
from sim import *
from dims import *
from math import *

def rotpos_to_vtkMatrix4x4(r,p):
    m = vtk.vtkMatrix4x4()
    m.DeepCopy((r[0], r[1], r[2], p[0],\
                r[3], r[4], r[5], p[1],\
                r[6], r[7], r[8], p[2],\
                0.0,  0.0,  0.0, 1.0))
    return m

class Robot:
    def __init__(self, sim, pos):
        global allGroups
        x, y, z = pos
        # Connect robot to simulation and vice-versa
        self.sim = sim
        self.sim.addRobot(self)
        # Create the legs
        self.rightLeg = Leg(sim,(x, y, z), 1.0)
        self.leftLeg = Leg(sim,(x, y, z), -1.0)
        # Create the trunk
        self.trunk = Link(sim,\
            (x+TRUNK_JPOS[0],\
             y+FOOT_LY+LLEG_LY+ULEG_LY+TRUNK_LY/2.0+TRUNK_JPOS[1],\
             z+TRUNK_JPOS[2]),\
            (TRUNK_LX, TRUNK_LY, TRUNK_LZ),\
            DENSITY)
        # Create left hip joints
        self.lhipPosition = (x-1*(FOOT_JPOS[0]+LEG_GAP/2.0),\
                             FOOT_LY+LLEG_LY+ULEG_LY+y,\
                             z+TRUNK_JPOS[2])
        self.lhipJointBody1 = ode.Body(sim.world) # intermediate massless body
        self.nullMass1 = ode.Mass(sim.world)
        self.nullMass1.setZero()
        self.nullMass1.setBox(0.01, 0.01, 0.01, 0.01)
        self.lhipJointBody1.setMass(self.nullMass1)
        self.lhipJointBody2 = ode.Body(sim.world) # intermediate massless body
        self.nullMass2 = ode.Mass(sim.world)
        self.nullMass2.setZero()
        self.nullMass2.setBox(0.01, 0.01, 0.01, 0.01)
        self.lhipJointBody2.setMass(self.nullMass2)
        self.lhipTilt = ode.HingeJoint(sim.world) # tilt
        self.lhipTilt.attach(self.leftLeg.uleg.body, self.lhipJointBody1)
        self.lhipTilt.setAnchor(self.lhipPosition)
        self.lhipTilt.setAxis((1,0,0))
        self.lhipRoll = ode.HingeJoint(sim.world) # roll
        self.lhipRoll.attach(self.lhipJointBody1, self.lhipJointBody2)
        self.lhipRoll.setAnchor(self.lhipPosition)
        self.lhipRoll.setAxis((0,0,1))
        self.lhipYaw = ode.HingeJoint(sim.world) # yaw
        self.lhipYaw.attach(self.lhipJointBody2, self.trunk.body)
        self.lhipYaw.setAnchor(self.lhipPosition)
        self.lhipYaw.setAxis((0,1,0))
        # Create right hip joints
        self.rhipPosition = (x+1*(FOOT_JPOS[0]+LEG_GAP/2.0),\
                             FOOT_LY+LLEG_LY+ULEG_LY+y,\
                             z+TRUNK_JPOS[2])
        self.rhipJointBody1 = ode.Body(sim.world) # intermediate massless body
        self.nullMass3 = ode.Mass(sim.world)
        self.nullMass3.setZero()
        self.nullMass3.setBox(0.01, 0.01, 0.01, 0.01)
        self.rhipJointBody1.setMass(self.nullMass3)
        self.rhipJointBody2 = ode.Body(sim.world) # intermediate massless body
        self.nullMass4 = ode.Mass(sim.world)
        self.nullMass4.setZero()
        self.nullMass4.setBox(0.01, 0.01, 0.01, 0.01)
        self.rhipJointBody2.setMass(self.nullMass4)
        self.rhipTilt = ode.HingeJoint(sim.world) # tilt
        self.rhipTilt.attach(self.rightLeg.uleg.body, self.rhipJointBody1)
        self.rhipTilt.setAnchor(self.rhipPosition)
        self.rhipTilt.setAxis((1,0,0))
        self.rhipRoll = ode.HingeJoint(sim.world) # roll
        self.rhipRoll.attach(self.rhipJointBody1, self.rhipJointBody2)
        self.rhipRoll.setAnchor(self.rhipPosition)
        self.rhipRoll.setAxis((0,0,1))
        self.rhipYaw = ode.HingeJoint(sim.world) # yaw
        self.rhipYaw.attach(self.rhipJointBody2, self.trunk.body)
        self.rhipYaw.setAnchor(self.rhipPosition)
        self.rhipYaw.setAxis((0,1,0))
        # No touch groups
        allGroups.append([self.leftLeg.uleg.body,self.trunk.body])
        allGroups.append([self.rightLeg.uleg.body,self.trunk.body])
        # Create the left hip tilt motor
        self.lhipTiltMotor = ode.AMotor(sim.world)
        self.lhipTiltMotor.attach(self.leftLeg.uleg.body, self.trunk.body)
        self.lhipTiltMotor.setNumAxes(1)
        self.lhipTiltMotor.setAxis(0,1,(1,0,0))
        self.lhipTiltMotor.enable()
        self.lhipTiltServo = ServoPID(self.lhipTiltMotor, self.lhipTilt)
        self.lhipTiltServo.setGains(HIPTILT_PID)
        self.lhipTiltServo.setMaxTorque(MAX_HIP_TORQUE)
        # Create the right hip tilt motor
        self.rhipTiltMotor = ode.AMotor(sim.world)
        self.rhipTiltMotor.attach(self.rightLeg.uleg.body, self.trunk.body)
        self.rhipTiltMotor.setNumAxes(1)
        self.rhipTiltMotor.setAxis(0,1,(1,0,0))
        self.rhipTiltMotor.enable()
        self.rhipTiltServo = ServoPID(self.rhipTiltMotor, self.rhipTilt)
        self.rhipTiltServo.setGains(HIPTILT_PID)
        self.rhipTiltServo.setMaxTorque(MAX_HIP_TORQUE)
        # Create the left hip roll motor
        self.lhipRollMotor = ode.AMotor(sim.world)
        self.lhipRollMotor.attach(self.leftLeg.uleg.body, self.trunk.body)
        self.lhipRollMotor.setNumAxes(1)
        self.lhipRollMotor.setAxis(0,1,(0,0,1))
        self.lhipRollMotor.enable()
        self.lhipRollServo = ServoPID(self.lhipRollMotor, self.lhipRoll)
        self.lhipRollServo.setGains(HIPROLL_PID)
        self.lhipRollServo.setMaxTorque(MAX_HIP_TORQUE)
        # Create the right hip roll motor
        self.rhipRollMotor = ode.AMotor(sim.world)
        self.rhipRollMotor.attach(self.rightLeg.uleg.body, self.trunk.body)
        self.rhipRollMotor.setNumAxes(1)
        self.rhipRollMotor.setAxis(0,1,(0,0,1))
        self.rhipRollMotor.enable()
        self.rhipRollServo = ServoPID(self.rhipRollMotor, self.rhipRoll)
        self.rhipRollServo.setGains(HIPROLL_PID)
        self.rhipRollServo.setMaxTorque(MAX_HIP_TORQUE)
        # Create the left hip yaw motor
        self.lhipYawMotor = ode.AMotor(sim.world)
        self.lhipYawMotor.attach(self.leftLeg.uleg.body, self.trunk.body)
        self.lhipYawMotor.setNumAxes(1)
        self.lhipYawMotor.setAxis(0,1,(0,1,0))
        self.lhipYawMotor.enable()
        self.lhipYawServo = ServoPID(self.lhipYawMotor, self.lhipYaw)
        self.lhipYawServo.setGains(HIPYAW_PID)
        self.lhipYawServo.setMaxTorque(MAX_HIP_TORQUE)
        # Create the right hip yaw motor
        self.rhipYawMotor = ode.AMotor(sim.world)
        self.rhipYawMotor.attach(self.rightLeg.uleg.body, self.trunk.body)
        self.rhipYawMotor.setNumAxes(1)
        self.rhipYawMotor.setAxis(0,1,(0,1,0))
        self.rhipYawMotor.enable()
        self.rhipYawServo = ServoPID(self.rhipYawMotor, self.rhipYaw)
        self.rhipYawServo.setGains(HIPYAW_PID)
        self.rhipYawServo.setMaxTorque(MAX_HIP_TORQUE)
    def updatePh(self, timeStep):
        self.rightLeg.updatePh(timeStep)
        self.leftLeg.updatePh(timeStep)
        self.lhipTiltServo.updatePh(timeStep)
        self.rhipTiltServo.updatePh(timeStep)
        self.lhipRollServo.updatePh(timeStep)
        self.rhipRollServo.updatePh(timeStep)
        self.lhipYawServo.updatePh(timeStep)
        self.rhipYawServo.updatePh(timeStep)
    def getPosition(self):
        return self.trunk.body.getPosition()
    def setLeftHipRoll(self,target):
        self.lhipRollServo.setTarget(-target)
    def setRightHipRoll(self,target):
        self.rhipRollServo.setTarget(target)
    def setLeftHipTilt(self,target):
        self.lhipTiltServo.setTarget(-target)
    def setRightHipTilt(self,target):
        self.rhipTiltServo.setTarget(-target)
    def setLeftHipYaw(self,target):
        self.lhipYawServo.setTarget(-target)
    def setRightHipYaw(self,target):
        self.rhipYawServo.setTarget(target)
    def setLeftKnee(self,target):
        self.leftLeg.kneeServo.setTarget(-target)
    def setRightKnee(self,target):
        self.rightLeg.kneeServo.setTarget(-target)
    def setLeftAnkleRoll(self,target):
        self.leftLeg.ankleRollServo.setTarget(-target)
    def setRightAnkleRoll(self,target):
        self.rightLeg.ankleRollServo.setTarget(target)
    def setLeftAnkleTilt(self,target):
        self.leftLeg.ankleTiltServo.setTarget(-target)
    def setRightAnkleTilt(self,target):
        self.rightLeg.ankleTiltServo.setTarget(-target)
    def getLeftHipRoll(self):
        return -self.lhipRollServo.readAngle()
    def getRightHipRoll(self):
        return self.rhipRollServo.readAngle()
    def getLeftHipTilt(self):
        return -self.lhipTiltServo.readAngle()
    def getRightHipTilt(self):
        return -self.rhipTiltServo.readAngle()
    def getLeftHipYaw(self):
        return -self.lhipYawServo.readAngle()
    def getRightHipYaw(self):
        return self.rhipYawServo.readAngle()
    def getLeftKnee(self):
        return -self.leftLeg.kneeServo.readAngle()
    def getRightKnee(self):
        return -self.rightLeg.kneeServo.readAngle()
    def getLeftAnkleRoll(self):
        return -self.leftLeg.ankleRollServo.readAngle()
    def getRightAnkleRoll(self):
        return self.rightLeg.ankleRollServo.readAngle()
    def getLeftAnkleTilt(self):
        return -self.leftLeg.ankleTiltServo.readAngle()
    def getRightAnkleTilt(self):
        return -self.rightLeg.ankleTiltServo.readAngle()
    def setTargets(self, targets):
        self.setLeftHipRoll(targets[0]) 
        self.setRightHipRoll(targets[1]) 
        self.setLeftHipTilt(targets[2]) 
        self.setRightHipTilt(targets[3]) 
        self.setLeftHipYaw(targets[4]) 
        self.setRightHipYaw(targets[5]) 
        self.setLeftKnee(targets[6]) 
        self.setRightKnee(targets[7]) 
        self.setLeftAnkleRoll(targets[8]) 
        self.setRightAnkleRoll(targets[9]) 
        self.setLeftAnkleTilt(targets[10]) 
        self.setRightAnkleTilt(targets[11]) 
    def readIMU(self):
        return self.trunk.readIMU()
    def readSensors(self):
        return [self.getLeftHipRoll(),\
                self.getRightHipRoll(),\
                self.getLeftHipTilt(),\
                self.getRightHipTilt(),\
                self.getLeftHipYaw(),\
                self.getRightHipYaw(),\
                self.getLeftKnee(),\
                self.getRightKnee(),\
                self.getLeftAnkleRoll(),\
                self.getRightAnkleRoll(),\
                self.getLeftAnkleTilt(),\
                self.getRightAnkleTilt(),\
                self.readIMU()]

class Leg:
    def __init__(self, sim, pos, right=1.0):
        global allGroups
        x, y, z = pos
        self.sim = sim
        # Create the foot
        self.foot = Link(self.sim,\
            (x+right*(FOOT_JPOS[0] + LEG_GAP/2.0),\
             y+FOOT_LY/2.0,\
             z+FOOT_JPOS[2]),\
            (FOOT_LX, FOOT_LY, FOOT_LZ),\
            DENSITY)
        # Create the lower leg
        self.lleg = Link(self.sim,\
            (x+right*(LLEG_JPOS[0] + LEG_GAP/2.0),\
             y+FOOT_LY+LLEG_LY/2.0+LLEG_JPOS[1],\
             z+LLEG_JPOS[2]),\
            (LLEG_LX, LLEG_LY, LLEG_LZ),\
            DENSITY)
        # Create the upper leg
        self.uleg = Link(self.sim,\
            (x+right*(ULEG_JPOS[0] + LEG_GAP/2.0),\
             y+FOOT_LY+LLEG_LY+ULEG_LY/2.0+ULEG_JPOS[1],\
             z+ULEG_JPOS[2]),\
            (ULEG_LX, ULEG_LY, ULEG_LZ),\
            DENSITY)
        # Create the ankle tilt and roll joints
        self.ankleJointBody1 = ode.Body(self.sim.world) # intermediate massless body
        self.nullMass = ode.Mass(self.sim.world)
        self.nullMass.setZero()
        self.nullMass.setBox(0.01, 0.01, 0.01, 0.01)
        self.ankleJointBody1.setMass(self.nullMass)
        self.anklePosition = (x+right*(FOOT_JPOS[0]+LEG_GAP/2.0),\
                              FOOT_LY/2.0+y,\
                              z+LLEG_JPOS[2])
        self.ankleJointBody1.setPosition(self.anklePosition)
        self.ankleTilt = ode.HingeJoint(self.sim.world) # ankle tilt joint
        self.ankleTilt.attach(self.foot.body, self.ankleJointBody1) # foot to ml body
        self.ankleTilt.setAnchor(self.anklePosition)
        self.ankleTilt.setAxis((1,0,0))
        self.ankleRoll = ode.HingeJoint(self.sim.world) # ankle roll joint
        self.ankleRoll.attach(self.ankleJointBody1, self.lleg.body) # ml body to lo-leg
        self.ankleRoll.setAnchor(self.anklePosition)
        self.ankleRoll.setAxis((0,0,1))
        # No collision between foot and lower leg
        allGroups.append([self.foot.body,self.lleg.body])
        # Create the knee joint
        self.knee = ode.HingeJoint(self.sim.world)
        self.knee.attach(self.lleg.body, self.uleg.body)
        # No collision between lower and upper leg
        allGroups.append([self.lleg.body,self.uleg.body])
        self.knee.setAnchor((x+right*(FOOT_JPOS[0]+LEG_GAP/2.0),\
                             FOOT_LY+LLEG_LY+y,\
                             z+ULEG_JPOS[2]))
        # Create the ankle motors
        self.ankleTiltMotor = ode.AMotor(self.sim.world)
        self.ankleTiltMotor.attach(self.foot.body, self.lleg.body)
        self.ankleTiltMotor.setNumAxes(1)
        self.ankleTiltMotor.setAxis(0,1,(1,0,0))
        self.ankleTiltMotor.enable()
        self.ankleTiltServo = ServoPID(self.ankleTiltMotor, self.ankleTilt)
        self.ankleTiltServo.setGains(ANKLE_PID)
        self.ankleTiltServo.setMaxTorque(MAX_ANKLE_TORQUE)
        self.ankleRollMotor = ode.AMotor(self.sim.world)
        self.ankleRollMotor.attach(self.foot.body, self.lleg.body)
        self.ankleRollMotor.setNumAxes(1)
        self.ankleRollMotor.setAxis(0,1,(0,0,1))
        self.ankleRollMotor.enable()
        self.ankleRollServo = ServoPID(self.ankleRollMotor, self.ankleRoll)
        self.ankleRollServo.setGains(ANKLE_PID)
        self.ankleRollServo.setMaxTorque(MAX_ANKLE_TORQUE)
        # Create the knee motors
        self.kneeMotor = ode.AMotor(self.sim.world)
        self.kneeMotor.attach(self.lleg.body, self.uleg.body)
        self.kneeMotor.setNumAxes(1)
        self.kneeMotor.setAxis(0,1,(1,0,0))
        self.kneeMotor.enable()
        self.kneeServo = ServoPID(self.kneeMotor, self.knee)
        self.kneeServo.setGains(KNEE_PID)
        self.kneeServo.setMaxTorque(MAX_KNEE_TORQUE)
    def updatePh(self, timeStep):
        self.ankleTiltServo.updatePh(timeStep)
        self.ankleRollServo.updatePh(timeStep)
        self.kneeServo.updatePh(timeStep)
        #print '%+04.0f %+04.0f' % (self.ankleTilt.getAngle()*180/pi, self.ankleRoll.getAngle()*180/pi)

class Link:
    def __init__(self, sim, pos, dims, density):
        # ODE initialization
        x, y, z = pos # initial pos
        lx, ly, lz = dims # dimensions
        self.sim = sim # link to the sim object
        self.body = ode.Body(self.sim.world) # ode body
        mass = ode.Mass() # mass object
        mass.setBox(density, lx, ly, lz) # calculate mass
        self.body.setMass(mass) # link mass to body
        self.body.setPosition(pos) # set the initial pos
        self.geom = ode.GeomBox(self.sim.space, lengths=dims) # geometry
        self.geom.setBody(self.body) # link geometry and body
        # VTK initialization
        self.math = vtk.vtkMath()
        self.cube = vtk.vtkCubeSource()
        self.cube.SetXLength(lx)
        self.cube.SetYLength(ly)
        self.cube.SetZLength(lz)
        self.cube.SetCenter((0.0,0.0,0.0))
        self.reader = vtk.vtkJPEGReader()
        self.reader.SetFileName(ROBOT_IMAGE)
        self.texture = vtk.vtkTexture()
        transform = vtk.vtkTransform()
        transform.Scale(1.0,1.0,1.0)
        self.texture.SetTransform(transform)
        self.texture.SetInput(self.reader.GetOutput())
        self.mapper = vtk.vtkPolyDataMapper()
        self.mapper.SetInput(self.cube.GetOutput())
        self.actor = vtk.vtkActor()
        self.actor.SetMapper(self.mapper)
        self.actor.SetTexture(self.texture)
        sim.renderer.AddActor(self.actor)
        # Self-include in the bodies for visualization
        sim.bodies.append(self)
    def readIMU(self):
        return self.body.vectorFromWorld((0,-1,0))
    def updateVi(self):
        # Called to update position and orientation
        # on the screen
        rot = self.body.getRotation()
        pos = self.body.getPosition()
        m = rotpos_to_vtkMatrix4x4(rot,pos)
        self.actor.PokeMatrix(m)

class ServoPID:
    def __init__(self, motor, joint):
        self.motor = motor
        #self.motor.setParam(ode.ParamFudgeFactor,0.5)
        self.joint = joint
        self.target = 0.0
        self.error = 0.0
        self.error_before = 0.0
        self.error_delta = 0.0
        self.error_sum = 0.0
        self.kp = 0.0001
        self.ki = 0.00005
        self.kd = 0.0000015
        self.output = [0.0, 0.0, 0.0]
        self.maxTorque = 1.0
    def setGains(self, ks):
        kp, ki, kd = ks
        self.kp = kp
        self.ki = ki
        self.kd = kd
    def setTarget(self, target):
        self.target = target
    def readAngle(self):
        return self.joint.getAngle()
    def updatePh(self, timeStep):
        x = self.joint.getAngle()
        self.error = self.target - x
        self.error_delta = (self.error - self.error_before) / timeStep
        self.error_before = self.error
        self.error_sum = self.error_sum + timeStep*self.error
        p = self.kp*self.error
        i = self.ki*self.error_sum
        d = self.kd*self.error_delta
        self.output = [p, i, d]
        torque = sum(self.output)
        if torque > self.maxTorque:
            torque = self.maxTorque
        self.motor.addTorques(torque, 0.0, 0.0)
    def setMaxTorque(self, maxTorque):
        self.maxTorque = maxTorque

