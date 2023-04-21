"""lab2_task2 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Measurements and constants
pi = 3.141593
wheelRadius = 1.6 / 2.0
wheelSeparation = 2.28

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
frontDistanceSensor = robot.getDevice('front_ds')
leftDistanceSensor = robot.getDevice('left_ds')
rightDistanceSensor = robot.getDevice('right_ds')
frontDistanceSensor.enable(timestep)
leftDistanceSensor.enable(timestep)
rightDistanceSensor.enable(timestep)


# enable camera and recognition
camera = robot.getDevice('camera1')
camera.enable(timestep)
camera.recognitionEnable(timestep)

# get handler to motors and set target position to infinity
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

# Variables
Kp = 2
Cmax = 2
Cmin = -2
R = 0

# Robot functions
def setAngularSpeed(o):
    leftMotor.setVelocity((o * wheelSeparation / 2) / wheelRadius)
    rightMotor.setVelocity(-(o * wheelSeparation / 2) / wheelRadius)

def E(x):
    return R - x

def U(E):
    return Kp * E

def fSat(vel):
    if vel > Cmax:
        return Cmax
    elif vel < Cmin:
        return Cmin
    else: 
        return vel


def Ur(x):
    return fSat(U(E(x))) 

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    print(not camera.getRecognitionObjects())
    if not camera.getRecognitionObjects():
        setAngularSpeed(2)
    else:
        target = camera.getRecognitionObjects()[0]
        print(target.get_position(), target.get_orientation())
        setAngularSpeed(Ur(target.get_position()[1]))

    print("Front",frontDistanceSensor.getValue()*39.97)
    print("Left",leftDistanceSensor.getValue()*39.97)
    print("Right",rightDistanceSensor.getValue()*39.97)
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
