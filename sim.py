import ode
import vtk
from dims import *

# Bodies that are connected should be
# included in the same group so that
# collisions are ignored.
allGroups = []

PERIOD = 1.0 / FRAMERATE
STEPS_PER_FRAME = max(int(PERIOD / TIME_STEP),1)

print "Period:",PERIOD
print "Steps per frame:", STEPS_PER_FRAME

# Main simulation class
class Sim:
    def __init__(self):
        # ODE initialization
        self.world = ode.World()
        self.world.setGravity(GRAVITY)
        self.world.setERP(ERP)
        self.world.setCFM(CFM)
        self.space = ode.Space()
        self.floor = ode.GeomPlane(self.space, (0.0,1.0,0.0), 0.0)
        self.jointGroup = ode.JointGroup()
        self.time = 0.0
        # VTK initialization
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(102.0/255.0,204/255.0,1.0)
        self.window = vtk.vtkRenderWindow()
        self.window.SetSize(WINDOW_WIDTH,WINDOW_HEIGHT)
        self.window.AddRenderer(self.renderer)
        self.interactor = vtk.vtkRenderWindowInteractor()
        self.interactor.SetRenderWindow(self.window)
        self.axes = vtk.vtkAxesActor()
        self.axes.SetAxisLabels(0)
        transform = vtk.vtkTransform()
        transform.Scale(0.1,0.1,0.1)
        self.axes.SetUserTransform(transform)
        self.renderer.AddActor(self.axes)
        # Create ground plane visualization
        self.floorVisual = vtk.vtkPlaneSource()
        self.floorVisual.SetNormal((0.0,1.0,0.0))
        self.floorVisual.SetResolution(10,10)
        self.floorReader = vtk.vtkJPEGReader()
        self.floorReader.SetFileName(FLOOR_IMAGE)
        self.floorTexture = vtk.vtkTexture()
        transform = vtk.vtkTransform()
        transform.Scale(50.0,50.0,50.0)
        self.floorTexture.SetTransform(transform)
        self.floorTexture.SetInput(self.floorReader.GetOutput())
        self.floorMap = vtk.vtkTextureMapToPlane()
        self.floorMap.SetInput(self.floorVisual.GetOutput())
        self.floorMapper = vtk.vtkPolyDataMapper()
        self.floorMapper.SetInput(self.floorMap.GetOutput())
        self.floorActor = vtk.vtkActor()
        transform = vtk.vtkTransform()
        transform.Scale(100.0,100.0,100.0)
        self.floorActor.SetUserTransform(transform)
        self.floorActor.SetMapper(self.floorMapper)
        self.floorActor.SetTexture(self.floorTexture)
        self.renderer.AddActor(self.floorActor)
        # VTK camera setup
        self.camera = vtk.vtkCamera()
        self.renderer.SetActiveCamera(self.camera)
        self.cameraFocus = [0.0, 0.0, 0.0]
        self.cameraPos = [4.0, 2.5, 1.5]
        self.cameraOffset = [3.0,2.0,1.0]
        self.cameraRoll = 0.0
        # Keep track of the simulated bodies and robots
        self.bodies = []
        self.robots = []
        self.controllers = []
    def addRobot(self, robot):
        self.robots.append(robot)
    def addController(self, controller):
        self.controllers.append(controller)
    def start(self):
        # Start the simulation -- to be called
        # after all the objects are created
        self.camera.SetPosition(self.cameraOffset)
        self.camera.SetFocalPoint(self.cameraPos)
        self.interactor.Initialize()
        self.interactor.AddObserver('TimerEvent', self.updateViPh)
        self.timerId = self.interactor.CreateRepeatingTimer(int(PERIOD*1000))
        self.window.Render()
        self.interactor.Start()
        groups = []
        global allGroups
        for group in allGroups:
            groups = groups + group
        allGroups = groups
    def adjustCamera(self, pos):
        self.cameraPos = list(self.camera.GetPosition())
        self.cameraFocus = list(self.camera.GetFocalPoint())
        for i in range(3):
            e = pos[i]+self.cameraOffset[i] - self.cameraPos[i]
            self.cameraPos[i] = self.cameraPos[i] + 0.005*e
            self.camera.SetPosition(self.cameraPos)
            e = pos[i] - self.cameraFocus[i]
            self.cameraFocus[i] = self.cameraFocus[i] + 0.025*e
            self.camera.SetFocalPoint(self.cameraFocus)
        roll = self.camera.GetRoll()
        e = self.cameraRoll - roll
        self.camera.SetRoll(roll + 0.05*e)
    def updatePh(self, timeStep=0.0001):
        # Update robot controllers
        for robot in self.robots:
            robot.updatePh(timeStep)
        for controller in self.controllers:
            controller.updatePh(timeStep)
        # Simulation step (physics only)
        self.space.collide((self.world, self.jointGroup), near_callback)
        self.world.step(timeStep)
        self.jointGroup.empty()
        self.time = self.time + timeStep
    def updateViPh(self, obj, event):
        # Called every frame this includes
        # physics simulation and visualization
        for i in range(STEPS_PER_FRAME):
            self.updatePh(TIME_STEP)
        for b in self.bodies:
            b.updateVi()
        if len(self.robots) > 0:
            self.adjustCamera(self.robots[0].getPosition())
        self.window.Render()

# This function is called when there is
# contact between two geometries
def near_callback(args, geom1, geom2):
    body1 = geom1.getBody()
    body2 = geom2.getBody()
    # Contacts in the same group are ignored
    # e.g. foot and lower leg
    for group in allGroups:
        if body1 in group and body2 in group:
            return
    # Create temporary contact for collisions
    contacts = ode.collide(geom1, geom2)
    world,contactgroup = args
    for c in contacts[:3]:
        c.setBounce(BOUNCE)
        c.setMu(MU)
        j = ode.ContactJoint(world, contactgroup, c)
        j.attach(body1, body2)

class Controller(object):
    def __init__(self, sim, robot):
        self.sim = sim
        self.robot = robot
        self.state = 0
        self.sim.addController(self)
    def updatePh(self,timeStep):
        pass

