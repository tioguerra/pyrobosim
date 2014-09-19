# Configuration parameters for the
# simulation

# Density in kg/m3
DENSITY = 500

# All lengths in meters
LEG_GAP = 0.180
FOOT_LX = 0.100
FOOT_LZ = 0.180
FOOT_LY = 0.020
FOOT_JPOS = (0.0, 0.0, FOOT_LZ/4.0)
LLEG_LX = 0.080
LLEG_LZ = 0.040
LLEG_LY = 0.300
LLEG_JPOS = (0.0, 0.0, 0.0)
ULEG_LX = 0.080
ULEG_LZ = 0.040
ULEG_LY = 0.260
ULEG_JPOS = (0.0, 0.0, 0.0)
TRUNK_LX = 0.280
TRUNK_LZ = 0.100
TRUNK_LY = 0.350
TRUNK_JPOS = (0.0, 0.0, 0.0)

# Gravity
GRAVITY = (0, -9.81, 0)
ERP = 0.8 # make this as large as possible
CFM = 0.001 # make this as small as possible
BOUNCE = 0.1
MU = 10.0

WINDOW_WIDTH = 1024
WINDOW_HEIGHT = 768

FLOOR_IMAGE = 'images/floor.jpg'
ROBOT_IMAGE = 'images/rough.jpg'

# Torque in N.m
MAX_KNEE_TORQUE = 100.0
MAX_ANKLE_TORQUE = 100.0
MAX_HIP_TORQUE = 100.0

EPS = 2e-52 # http://www.mathworks.com/help/techdoc/ref/eps.html

FRAMERATE = 60.0
TIME_STEP = 0.001

HIPROLL_PID = 25.0, 2.5, 1.5
HIPTILT_PID = 25.0, 2.5, 1.5
HIPYAW_PID = 15.0, 1.5, 0.25
KNEE_PID = 25.0, 2.5, 1.5
ANKLE_PID = 15.0, 1.5, 0.25