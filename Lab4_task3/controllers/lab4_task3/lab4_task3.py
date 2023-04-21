"""lab2_task2 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from turtle import left
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
Cmax = 3
Cmin = -3
r = -5
leftTurn = True

# States
findGoal = True
mg = False
wf = False
turn = False

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

def setAngularSpeed(o):
    leftMotor.setVelocity((o * wheelSeparation / 2) / wheelRadius)
    rightMotor.setVelocity(-(o * wheelSeparation / 2) / wheelRadius)

def E(x, R):
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


def Ur(x, R):
    return fSat(U(E(x, R))) 

def wallFollow(sensor, r):
    left = Cmax
    right = Cmax
    cor = Ur(getInches(sensor), r) * 0.5

    if leftTurn:
        if getInches(sensor) > 5.1:
            right -= abs(cor)
        elif getInches(sensor) < 4.9:
            left -= abs(cor)
    else:  
        if getInches(sensor) > 5.1:
            left -= abs(cor)
        elif getInches(sensor) < 4.9:
            right -= abs(cor)
    

    setSpeedsIPS(left, right)
    return

# Utility
def frontWall():
    return getInches(frontDistanceSensor) < 5.2
def leftWall():
    return getInches(leftDistanceSensor) < 5.2 
def rightWall():
    return getInches(rightDistanceSensor) < 5.1

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    if findGoal:
        mg = camera.getRecognitionObjects()
        turn = frontWall()
        if not mg and not turn:
            setAngularSpeed(2)
        else:
            findGoal = False
    elif mg:
        if camera.getRecognitionObjects():                                  
            print('Motion to goal')
            target = camera.getRecognitionObjects()[0]
            # Center Target
            setAngularSpeed(Ur(target.get_position()[1], 0))
            if target.get_position()[1] > -0.0001 and target.get_position()[1] < 0.0001:
                # Move towards target
                setSpeedsIPS(Ur(-1 * getInches(frontDistanceSensor), r), Ur(-1 * getInches(frontDistanceSensor), r))
            # State conditions
        else: 
            mg = False  
    elif turn:
        print('Turning')
        if leftTurn:
            setSpeedsIPS(-1.8, 1.8)
            # State conditions
            if rightWall() and not frontWall():
                wf = True
                turn = False
        else:
            setSpeedsIPS(2.5, -2.5)
            # State conditions
            if leftWall() and not frontWall():
                wf = True
                turn = False
    elif wf:
        print('Wall following')
        if leftTurn:
            wallFollow(rightDistanceSensor, -5)
        else:
            wallFollow(leftDistanceSensor, -5)
        # State conditions
        if frontWall():
            turn = True
            wf = False
        elif camera.getRecognitionObjects() and not (getInches(leftDistanceSensor) < 6 or getInches(rightDistanceSensor) < 6 or getInches(frontDistanceSensor) < 12):  # Clear line of sight
            wf = False
            mg = True
    else:
        print('Advancing')
        setSpeedsIPS(5, 5)
        # State conditions
        mg = camera.getRecognitionObjects() and not (getInches(leftDistanceSensor) < 6 or getInches(rightDistanceSensor) < 6 or getInches(frontDistanceSensor) < 12)  # Clear line of sight
        turn = frontWall()

    print("Front",frontDistanceSensor.getValue()*39.97)
    print("Left",leftDistanceSensor.getValue()*39.97)
    print("Right",rightDistanceSensor.getValue()*39.97)
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    print('---------------------------------------------------')
    pass

# Enter here exit cleanup code.
