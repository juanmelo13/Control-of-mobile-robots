"""lab1_task1 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Measurements and constants
pi = 3.141593
wheelRadius = 1.6 / 2.0
wheelSeparation = 2.28

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

#getting the position sensors

leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)

robot.step(timestep)

# Define functions
# def setSpeedsPWM(pwmLeft, pmwRight):
    # return
    

def setSpeedsRPS(rpsLeft, rpsRight):
    if (abs(rpsLeft) > 1 or abs(rpsRight) > 1):
        print('Target speed exceeds maximum speed!')
    else:        
        leftMotor.setVelocity(rpsLeft * 2 * pi)
        rightMotor.setVelocity(rpsRight *2 * pi)
    return

def setSpeedsIPS(ipsLeft, ipsRight):
    leftSpeed = ipsLeft / wheelRadius
    rightSpeed = ipsRight / wheelRadius
    print(str(leftSpeed))
    if (abs(leftSpeed) > 6.28 or abs(rightSpeed) > 6.28):
        print('Target speed exceeds maximum speed!')
    else:
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)
    return
    
def setSpeedsVW(V, W):
    if (W != 0):
        turnRadius = V / W
        ipsLeft = W * (turnRadius - wheelSeparation / 2)
        ipsRight = W * (turnRadius + wheelSeparation / 2)
        setSpeedsIPS(ipsLeft, ipsRight)
    else:
       setSpeedsIPS(V, V) 
    return

def setPositionI(X):
    leftMotor.setPosition(X/wheelRadius)
    rightMotor.setPosition(X/wheelRadius)
    
def moveXV(X, V):
    setSpeedsVW(V, 0)
    if (V < 0):
        X = -1 * X
    setPositionI(X)
    return

# Main loop:
# - perform simulation steps until Webots is stopping the controller -

t = 0.0    # Time elapsed (s)
x = 30.0    # Target Distance (in)
v = -5.0    # Target Velocity (in/s)

moveXV(x, v)

print("Stopping Time: " + str(abs(x)/abs(v)))

while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    
    # print("Left position sensor: " +str(leftposition_sensor.getValue()))
    # print("Right position sensor: " +str(rightposition_sensor.getValue()))
    print("Time elapsed: " + str(t))
    if (abs(v) * t > x):
        print("Distance Traveled: " + str(x))
    else:
        print("Distance Traveled: " + str(abs(v) * t))
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)

    t += timestep / 1000.0

# Enter here exit cleanup code.
