"""lab2_task1 controller."""

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
Kp = 5.0
Cmax = 6.279 * wheelRadius
Cmin = -6.279 * wheelRadius
R = -10

# Functions
def getInches(sensor):
    return sensor.getValue() * 39.37
    
def setSpeedsIPS(ipsLeft, ipsRight):
    leftSpeed = ipsLeft / wheelRadius
    rightSpeed = ipsRight / wheelRadius
    if (abs(leftSpeed) > 6.28 or abs(rightSpeed) > 6.28):
        print('Target speed exceeds maximum speed!')
    else:
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)
    return

def E():
    return R - getInches(frontDSensor) * -1

def U(E):
    return Kp * E

def fSat(vel):
    if vel > Cmax:
        return Cmax
    elif vel < Cmin:
        return Cmin
    else: 
        return vel


def Ur():
    return fSat(U(E())) 


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    print('Front: ' + str(getInches(frontDSensor)))
    print('Left: ' + str(getInches(leftDSensor)))
    print('Right: ' + str(getInches(rightDSensor)))
    
    setSpeedsIPS(Ur(), Ur())
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
