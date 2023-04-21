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

#initialization of motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

#distance sensors 
frontDSensor = robot.getDevice('front_ds')
leftDSensor = robot.getDevice('left_ds')
rightDSensor = robot.getDevice('right_ds')
frontDSensor.enable(timestep)
leftDSensor.enable(timestep)
rightDSensor.enable(timestep)

# Variables
Kp = 2.5
Cmax = 6.278 * wheelRadius
Cmin = -6.278 * wheelRadius
R = -3.7795

# Functions
def getInches(sensor):
    return sensor.getValue() * 39.37
    
def setSpeedsIPS(ipsLeft, ipsRight):
    leftSpeed = ipsLeft / wheelRadius
    rightSpeed = ipsRight / wheelRadius
    if (abs(leftSpeed) > 6.28 or abs(rightSpeed) > 6.28):
        print('Target speed exceeds maximum speed!')
        print(leftSpeed)
        print(rightSpeed)
    else:
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)
    return

def E(sensor):
    return R + getInches(sensor)

def U(E):
    return Kp * E

def fSat(vel):
    if vel > Cmax:
        return Cmax
    elif vel < Cmin:
        return Cmin
    else: 
        return vel

def Ur(sensor):
    return fSat(U(E(sensor))) 

def sideMotion(l, r):
    left = Cmax - l
    right = Cmax - r
    
    return (fSat(left), fSat(right))

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    print('Front: ' + str(getInches(frontDSensor)) + '; Left: ' + str(getInches(leftDSensor)) + '; Right: ' + str(getInches(rightDSensor)))
    print('--------------------------------------------------------------------------')

    
    # Process sensor data:
    speeds = sideMotion(Ur(leftDSensor), Ur(rightDSensor))
    setSpeedsIPS(speeds[0], speeds[1])
    pass

# Enter here exit cleanup code.
