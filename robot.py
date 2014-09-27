import ode
import vtk
from sim import *
from dims import *
from math import *
from util import *

# This class creates a biped robot
class QuadrupedRobot:
    def __init__(self, sim, pos, fixed=False):
        global allGroups
        x, y, z = pos
        # Connect robot to simulation and vice-versa
        self.sim = sim
        self.sim.addRobot(self)
        # Create the legs
        self.frontRightLeg = QuadrupedLeg(sim,(x, y, z+QTRUNK_LZ/2-QULEG_LZ/2), 1.0)
        self.frontLeftLeg = QuadrupedLeg(sim,(x, y, z+QTRUNK_LZ/2-QULEG_LZ/2), -1.0)
        self.backRightLeg = QuadrupedLeg(sim,(x, y, z-QTRUNK_LZ/2+QULEG_LZ/2), 1.0)
        self.backLeftLeg = QuadrupedLeg(sim,(x, y, z-QTRUNK_LZ/2+QULEG_LZ/2), -1.0)
        # Create the trunk
        self.trunk = Link(sim,\
            (x,y,z),\
            (QTRUNK_LX, QTRUNK_LY, QTRUNK_LZ),\
            DENSITY, fixed)
        # Create front left hip joints
        self.flhipPosition = (x-QTRUNK_LX/2,y,z+QTRUNK_LZ/2)
        self.flhipJointBody1 = ode.Body(sim.world) # intermediate massless body
        self.nullMass1 = ode.Mass(sim.world)
        self.nullMass1.setBox(0.01, 0.01, 0.01, 0.01)
        self.flhipJointBody1.setMass(self.nullMass1)
        self.flhipRoll = ode.HingeJoint(sim.world) # roll
        self.flhipRoll.attach(self.frontLeftLeg.uleg.body, self.flhipJointBody1)
        self.flhipRoll.setAnchor(self.flhipPosition)
        self.flhipRoll.setAxis((0,0,1))
        self.flhipYaw = ode.HingeJoint(sim.world) # yaw
        self.flhipYaw.attach(self.flhipJointBody1, self.trunk.body)
        self.flhipYaw.setAnchor(self.flhipPosition)
        self.flhipYaw.setAxis((0,1,0))
        # Create front right hip joints
        self.frhipPosition = (x+QTRUNK_LX/2,y,z+QTRUNK_LZ/2)
        self.frhipJointBody1 = ode.Body(sim.world) # intermediate massless body
        self.nullMass2 = ode.Mass(sim.world)
        self.nullMass2.setBox(0.01, 0.01, 0.01, 0.01)
        self.frhipJointBody1.setMass(self.nullMass2)
        self.frhipRoll = ode.HingeJoint(sim.world) # roll
        self.frhipRoll.attach(self.frontRightLeg.uleg.body, self.frhipJointBody1)
        self.frhipRoll.setAnchor(self.frhipPosition)
        self.frhipRoll.setAxis((0,0,1))
        self.frhipYaw = ode.HingeJoint(sim.world) # yaw
        self.frhipYaw.attach(self.frhipJointBody1, self.trunk.body)
        self.frhipYaw.setAnchor(self.frhipPosition)
        self.frhipYaw.setAxis((0,1,0))
        # Create back left hip joints
        self.blhipPosition = (x-QTRUNK_LX/2,y,z-QTRUNK_LZ/2)
        self.blhipJointBody1 = ode.Body(sim.world) # intermediate massless body
        self.nullMass3 = ode.Mass(sim.world)
        self.nullMass3.setBox(0.01, 0.01, 0.01, 0.01)
        self.blhipJointBody1.setMass(self.nullMass3)
        self.blhipRoll = ode.HingeJoint(sim.world) # roll
        self.blhipRoll.attach(self.backLeftLeg.uleg.body, self.blhipJointBody1)
        self.blhipRoll.setAnchor(self.blhipPosition)
        self.blhipRoll.setAxis((0,0,1))
        self.blhipYaw = ode.HingeJoint(sim.world) # yaw
        self.blhipYaw.attach(self.blhipJointBody1, self.trunk.body)
        self.blhipYaw.setAnchor(self.blhipPosition)
        self.blhipYaw.setAxis((0,1,0))
        # Create back right hip joints
        self.brhipPosition = (x+QTRUNK_LX/2,y,z-QTRUNK_LZ/2)
        self.brhipJointBody1 = ode.Body(sim.world) # intermediate massless body
        self.nullMass4 = ode.Mass(sim.world)
        self.nullMass4.setBox(0.01, 0.01, 0.01, 0.01)
        self.brhipJointBody1.setMass(self.nullMass4)
        self.brhipRoll = ode.HingeJoint(sim.world) # roll
        self.brhipRoll.attach(self.backRightLeg.uleg.body, self.brhipJointBody1)
        self.brhipRoll.setAnchor(self.brhipPosition)
        self.brhipRoll.setAxis((0,0,1))
        self.brhipYaw = ode.HingeJoint(sim.world) # yaw
        self.brhipYaw.attach(self.brhipJointBody1, self.trunk.body)
        self.brhipYaw.setAnchor(self.brhipPosition)
        self.brhipYaw.setAxis((0,1,0))
        # No touch groups
        allGroups.append([self.frontLeftLeg.uleg.body,self.trunk.body])
        allGroups.append([self.frontRightLeg.uleg.body,self.trunk.body])
        allGroups.append([self.backLeftLeg.uleg.body,self.trunk.body])
        allGroups.append([self.backRightLeg.uleg.body,self.trunk.body])
        # Create the front left hip roll motor
        self.flhipRollMotor = ode.AMotor(sim.world)
        self.flhipRollMotor.attach(self.frontLeftLeg.uleg.body, self.trunk.body)
        self.flhipRollMotor.setNumAxes(1)
        self.flhipRollMotor.setAxis(0,1,(0,0,1))
        self.flhipRollMotor.enable()
        self.flhipRollServo = ServoPID(self.flhipRollMotor, self.flhipRoll)
        self.flhipRollServo.setGains(QHIPROLL_PID)
        self.flhipRollServo.setMaxTorque(QMAX_HIP_TORQUE)
        # Create the front right hip roll motor
        self.frhipRollMotor = ode.AMotor(sim.world)
        self.frhipRollMotor.attach(self.frontRightLeg.uleg.body, self.trunk.body)
        self.frhipRollMotor.setNumAxes(1)
        self.frhipRollMotor.setAxis(0,1,(0,0,1))
        self.frhipRollMotor.enable()
        self.frhipRollServo = ServoPID(self.frhipRollMotor, self.frhipRoll)
        self.frhipRollServo.setGains(QHIPROLL_PID)
        self.frhipRollServo.setMaxTorque(QMAX_HIP_TORQUE)
        # Create the back left hip roll motor
        self.blhipRollMotor = ode.AMotor(sim.world)
        self.blhipRollMotor.attach(self.backLeftLeg.uleg.body, self.trunk.body)
        self.blhipRollMotor.setNumAxes(1)
        self.blhipRollMotor.setAxis(0,1,(0,0,1))
        self.blhipRollMotor.enable()
        self.blhipRollServo = ServoPID(self.blhipRollMotor, self.blhipRoll)
        self.blhipRollServo.setGains(QHIPROLL_PID)
        self.blhipRollServo.setMaxTorque(QMAX_HIP_TORQUE)
        # Create the back right hip roll motor
        self.brhipRollMotor = ode.AMotor(sim.world)
        self.brhipRollMotor.attach(self.backRightLeg.uleg.body, self.trunk.body)
        self.brhipRollMotor.setNumAxes(1)
        self.brhipRollMotor.setAxis(0,1,(0,0,1))
        self.brhipRollMotor.enable()
        self.brhipRollServo = ServoPID(self.brhipRollMotor, self.brhipRoll)
        self.brhipRollServo.setGains(QHIPROLL_PID)
        self.brhipRollServo.setMaxTorque(QMAX_HIP_TORQUE)
        # Create the front left hip yaw motor
        self.flhipYawMotor = ode.AMotor(sim.world)
        self.flhipYawMotor.attach(self.frontLeftLeg.uleg.body, self.trunk.body)
        self.flhipYawMotor.setNumAxes(1)
        self.flhipYawMotor.setAxis(0,1,(0,1,0))
        self.flhipYawMotor.enable()
        self.flhipYawServo = ServoPID(self.flhipYawMotor, self.flhipYaw)
        self.flhipYawServo.setGains(QHIPYAW_PID)
        self.flhipYawServo.setMaxTorque(QMAX_HIP_TORQUE)
        # Create the front right hip yaw motor
        self.frhipYawMotor = ode.AMotor(sim.world)
        self.frhipYawMotor.attach(self.frontRightLeg.uleg.body, self.trunk.body)
        self.frhipYawMotor.setNumAxes(1)
        self.frhipYawMotor.setAxis(0,1,(0,1,0))
        self.frhipYawMotor.enable()
        self.frhipYawServo = ServoPID(self.frhipYawMotor, self.frhipYaw)
        self.frhipYawServo.setGains(QHIPYAW_PID)
        self.frhipYawServo.setMaxTorque(QMAX_HIP_TORQUE)
        # Create the back left hip yaw motor
        self.blhipYawMotor = ode.AMotor(sim.world)
        self.blhipYawMotor.attach(self.backLeftLeg.uleg.body, self.trunk.body)
        self.blhipYawMotor.setNumAxes(1)
        self.blhipYawMotor.setAxis(0,1,(0,1,0))
        self.blhipYawMotor.enable()
        self.blhipYawServo = ServoPID(self.blhipYawMotor, self.blhipYaw)
        self.blhipYawServo.setGains(QHIPYAW_PID)
        self.blhipYawServo.setMaxTorque(QMAX_HIP_TORQUE)
        # Create the back right hip yaw motor
        self.brhipYawMotor = ode.AMotor(sim.world)
        self.brhipYawMotor.attach(self.backRightLeg.uleg.body, self.trunk.body)
        self.brhipYawMotor.setNumAxes(1)
        self.brhipYawMotor.setAxis(0,1,(0,1,0))
        self.brhipYawMotor.enable()
        self.brhipYawServo = ServoPID(self.brhipYawMotor, self.brhipYaw)
        self.brhipYawServo.setGains(QHIPYAW_PID)
        self.brhipYawServo.setMaxTorque(QMAX_HIP_TORQUE)
    def updatePh(self, timeStep):
        self.frontRightLeg.updatePh(timeStep)
        self.frontLeftLeg.updatePh(timeStep)
        self.backRightLeg.updatePh(timeStep)
        self.backLeftLeg.updatePh(timeStep)
        self.flhipRollServo.updatePh(timeStep)
        self.frhipRollServo.updatePh(timeStep)
        self.flhipYawServo.updatePh(timeStep)
        self.frhipYawServo.updatePh(timeStep)
        self.blhipRollServo.updatePh(timeStep)
        self.brhipRollServo.updatePh(timeStep)
        self.blhipYawServo.updatePh(timeStep)
        self.brhipYawServo.updatePh(timeStep)
    def getPosition(self):
        return self.trunk.body.getPosition()
    def setFrontLeftHipRoll(self,target):
        self.flhipRollServo.setTarget(-target)
    def setFrontRightHipRoll(self,target):
        self.frhipRollServo.setTarget(target)
    def setBackLeftHipRoll(self,target):
        self.blhipRollServo.setTarget(-target)
    def setBackRightHipRoll(self,target):
        self.brhipRollServo.setTarget(target)
    def setFrontLeftHipYaw(self,target):
        self.flhipYawServo.setTarget(-target)
    def setFrontRightHipYaw(self,target):
        self.frhipYawServo.setTarget(target)
    def setBackLeftHipYaw(self,target):
        self.blhipYawServo.setTarget(-target)
    def setBackRightHipYaw(self,target):
        self.brhipYawServo.setTarget(target)
    def setFrontLeftKnee(self,target):
        self.frontLeftLeg.kneeServo.setTarget(-target)
    def setFrontRightKnee(self,target):
        self.frontRightLeg.kneeServo.setTarget(-target)
    def setBackLeftKnee(self,target):
        self.backLeftLeg.kneeServo.setTarget(-target)
    def setBackRightKnee(self,target):
        self.backRightLeg.kneeServo.setTarget(-target)
    def setTargets(self, targets):
        if 'flHipRoll' in targets:
            self.setFrontLeftHipRoll(targets['flHipRoll'])
        if 'frHipRoll' in targets:
            self.setFrontRightHipRoll(targets['frHipRoll'])
        if 'blHipRoll' in targets:
            self.setBackLeftHipRoll(targets['blHipRoll'])
        if 'brHipRoll' in targets:
            self.setBackRightHipRoll(targets['brHipRoll'])
        if 'flHipYaw' in targets:
            self.setFrontLeftHipYaw(targets['flHipYaw'])
        if 'frHipYaw' in targets:
            self.setFrontRightHipYaw(targets['frHipYaw'])
        if 'blHipYaw' in targets:
            self.setBackLeftHipYaw(targets['blHipYaw'])
        if 'brHipYaw' in targets:
            self.setBackRightHipYaw(targets['brHipYaw'])
        if 'flKnee' in targets:
            self.setFrontLeftKnee(targets['flKnee'])
        if 'frKnee' in targets:
            self.setFrontRightKnee(targets['frKnee'])
        if 'blKnee' in targets:
            self.setBackLeftKnee(targets['blKnee'])
        if 'brKnee' in targets:
            self.setBackRightKnee(targets['brKnee'])
    def getFrontLeftHipRoll(self):
        return -self.flhipRollServo.readAngle()
    def getFrontRightHipRoll(self):
        return self.frhipRollServo.readAngle()
    def getBackLeftHipRoll(self):
        return -self.blhipRollServo.readAngle()
    def getBackRightHipRoll(self):
        return self.brhipRollServo.readAngle()
    def getFrontLeftHipYaw(self):
        return -self.flhipYawServo.readAngle()
    def getFrontRightHipYaw(self):
        return self.frhipYawServo.readAngle()
    def getBackLeftHipYaw(self):
        return -self.blhipYawServo.readAngle()
    def getBackRightHipYaw(self):
        return self.brhipYawServo.readAngle()
    def getFrontLeftKnee(self):
        return -self.frontLeftLeg.kneeServo.readAngle()
    def getFrontRightKnee(self):
        return -self.frontRightLeg.kneeServo.readAngle()
    def getBackLeftKnee(self):
        return -self.backLeftLeg.kneeServo.readAngle()
    def getBackRightKnee(self):
        return -self.backRightLeg.kneeServo.readAngle()
    def readIMU(self):
        return self.trunk.readIMU()
    def readTiltSensor(self):
        x,y,z = self.readIMU()
        return atan2(y,z)
    def readRollSensor(self):
        x,y,z = self.readIMU()
        return atan2(y,x)
    def readSensors(self):
        return {'flHipRoll':self.getFrontLeftHipRoll(),
                'frHipRoll':self.getFrontRightHipRoll(),
                'blHipRoll':self.getBackLeftHipRoll(),
                'brHipRoll':self.getBackRightHipRoll(),
                'flHipYaw':self.getFrontLeftHipYaw(),
                'frHipYaw':self.getFrontRightHipYaw(),
                'blKnee':self.getBackLeftHipYaw(),
                'brKnee':self.getBackRightHipYaw(),
                'flKnee':self.getFrontLeftKnee(),
                'frKnee':self.getFrontRightKnee(),
                'blKnee':self.getBackLeftKnee(),
                'brKnee':self.getBackRightKnee(),
                'tiltSensor':self.readTiltSensor(),
                'rollSensor':self.readRollSensor()}

# This class creates a biped robot
class BipedRobot:
    def __init__(self, sim, pos, fixed=False):
        global allGroups
        x, y, z = pos
        # Connect robot to simulation and vice-versa
        self.sim = sim
        self.sim.addRobot(self)
        # Create the legs
        self.rightLeg = BipedLeg(sim,(x, y, z), 1.0)
        self.leftLeg = BipedLeg(sim,(x, y, z), -1.0)
        # Create the trunk
        self.trunk = Link(sim,\
            (x+TRUNK_JPOS[0],\
             y+FOOT_LY+LLEG_LY+ULEG_LY+TRUNK_LY/2.0+TRUNK_JPOS[1],\
             z+TRUNK_JPOS[2]),\
            (TRUNK_LX, TRUNK_LY, TRUNK_LZ),\
            DENSITY, fixed)
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
        if 'lHipRoll' in targets:
            self.setLeftHipRoll(targets['lHipRoll']) 
        if 'rHipRoll' in targets:
            self.setRightHipRoll(targets['rHipRoll']) 
        if 'lHipTilt' in targets:
            self.setLeftHipTilt(targets['lHipTilt']) 
        if 'rHipTilt' in targets:
            self.setRightHipTilt(targets['rHipTilt']) 
        if 'lHipYaw' in targets:
            self.setLeftHipYaw(targets['lHipYaw']) 
        if 'rHipYaw' in targets:
            self.setRightHipYaw(targets['rHipYaw']) 
        if 'lKnee' in targets:
            self.setLeftKnee(targets['lKnee']) 
        if 'rKnee' in targets:
            self.setRightKnee(targets['rKnee']) 
        if 'lAnkleRoll' in targets:
            self.setLeftAnkleRoll(targets['lAnkleRoll']) 
        if 'rAnkleRoll' in targets:
            self.setRightAnkleRoll(targets['rAnkleRoll']) 
        if 'lAnkleTilt' in targets:
            self.setLeftAnkleTilt(targets['lAnkleTilt']) 
        if 'rAnkleTilt' in targets:
            self.setRightAnkleTilt(targets['rAnkleTilt']) 
    def readIMU(self):
        return self.trunk.readIMU()
    def readTiltSensor(self):
        x,y,z = self.readIMU()
        return atan2(y,z)
    def readRollSensor(self):
        x,y,z = self.readIMU()
        return atan2(y,x)
    def readSensors(self):
        return {'lHipRoll':self.getLeftHipRoll(),\
                'rHipRoll':self.getRightHipRoll(),\
                'lHipTilt':self.getLeftHipTilt(),\
                'rHipTilt':self.getRightHipTilt(),\
                'lHipYaw':self.getLeftHipYaw(),\
                'rHipYaw':self.getRightHipYaw(),\
                'lKnee':self.getLeftKnee(),\
                'rKnee':self.getRightKnee(),\
                'lAnkleRoll':self.getLeftAnkleRoll(),\
                'rAnkleRoll':self.getRightAnkleRoll(),\
                'lAnkleTilt':self.getLeftAnkleTilt(),\
                'rAnkleTilt':self.getRightAnkleTilt(),\
                'tiltSensor':self.readTiltSensor(),\
                'rollSensor':self.readRollSensor()}

class QuadrupedLeg:
    def __init__(self, sim, pos, right):
        global allGroups
        x, y, z = pos
        self.sim = sim
        # Create the foot
        self.foot = Link(self.sim,\
            (x+right*(QTRUNK_LX/2+QULEG_LX),\
             y-QLLEG_LY-QFOOT_LY/2,\
             z),\
            (2*QFOOT_RADIUS, QFOOT_LY, 2*QFOOT_RADIUS),\
            DENSITY)
        # Create the lower leg
        self.lleg = Link(self.sim,\
            (x+right*(QTRUNK_LX/2+QULEG_LX),\
             y-QLLEG_LY/2,\
             z),\
            (QLLEG_LX, QLLEG_LY, QLLEG_LZ),\
            DENSITY)
        # Create the upper leg
        self.uleg = Link(self.sim,\
            (x+right*(QTRUNK_LX/2+QULEG_LX/2),\
             y,\
             z),\
            (QULEG_LX, QULEG_LY, QULEG_LZ),\
            DENSITY)
        # Create the yaw joint
        self.anklePosition = (x+right*(QTRUNK_LX/2+QULEG_LX),\
                              y-QLLEG_LY-QFOOT_LY/2,\
                              z)
        self.ankle = ode.HingeJoint(self.sim.world) # ankle yaw joint
        self.ankle.attach(self.foot.body, self.lleg.body) # foot to lleg
        self.ankle.setAnchor(self.anklePosition)
        self.ankle.setAxis((0,1,0))
        # No collision between foot and lower leg
        allGroups.append([self.foot.body,self.lleg.body])
        # Create the knee joint
        self.kneePosition = (x+right*(QTRUNK_LX/2+QULEG_LX),y,z)
        self.knee = ode.HingeJoint(self.sim.world)
        self.knee.attach(self.lleg.body, self.uleg.body)
        self.knee.setAnchor(self.kneePosition)
        self.knee.setAxis((0,0,1))
        # No collision between lower and upper leg
        allGroups.append([self.lleg.body,self.uleg.body])
        # Create the knee motors
        self.kneeMotor = ode.AMotor(self.sim.world)
        self.kneeMotor.attach(self.lleg.body, self.uleg.body)
        self.kneeMotor.setNumAxes(1)
        self.kneeMotor.setAxis(0,1,(0,0,1))
        self.kneeMotor.enable()
        self.kneeServo = ServoPID(self.kneeMotor, self.knee)
        self.kneeServo.setGains(QKNEE_PID)
        self.kneeServo.setMaxTorque(QMAX_KNEE_TORQUE)
    def updatePh(self, timeStep):
        self.kneeServo.updatePh(timeStep)

class BipedLeg:
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

class Link:
    def __init__(self, sim, pos, dims, density, fixed=False):
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
        if fixed:
            self.fixedJoint = ode.FixedJoint(self.sim.world)
            self.fixedJoint.attach(self.body,self.sim.space.getBody())
            self.fixedJoint.setFixed()
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

