from controller import Robot, DistanceSensor, Motor, LightSensor

# Simulation time step in milliseconds
TIME_STEP = 64
MAX_SPEED = 6.28

# Create the robot instance
robot = Robot()

# Initialize distance sensors
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]
ls = []
lsNames = [
    'ls0', 'ls1', 'ls2', 'ls3',
    'ls4', 'ls5', 'ls6', 'ls7'
]

for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ls.append(robot.getDevice(lsNames[i]))
    ps[i].enable(TIME_STEP)
    ls[i].enable(TIME_STEP)

# Initialize motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

### used AI to help make these turning functions. Realized needed to time the turns,
### but didnt know what the syntax was for webots/python
### I knew i needed to turn wheels for certain amount of time just didnt know how
def turn_right():
    start_time = robot.getTime() * 1000  # Convert to ms
    while (robot.getTime() * 1000 - start_time) < 500:
        leftMotor.setVelocity(0.5 * MAX_SPEED)
        rightMotor.setVelocity(-0.5 * MAX_SPEED)
        robot.step(TIME_STEP)
    leftMotor.setVelocity(0.0)
    rightMotor.setVelocity(0.0)

def turn_left():
    start_time = robot.getTime() * 1000  # Convert to ms
    while (robot.getTime() * 1000 - start_time) < 500:
        leftMotor.setVelocity(-0.5 * MAX_SPEED)
        rightMotor.setVelocity(0.5 * MAX_SPEED)
        robot.step(TIME_STEP)
    leftMotor.setVelocity(0.0)
    rightMotor.setVelocity(0.0)
    
def turn_180():
    print("Turning 180 degrees")
    start_time = robot.getTime() * 1000  # Convert to ms
    while (robot.getTime() * 1000 - start_time) < 1500:
        leftMotor.setVelocity(-0.5 * MAX_SPEED)
        rightMotor.setVelocity(0.5 * MAX_SPEED)
        robot.step(TIME_STEP)
    leftMotor.setVelocity(0.0)
    rightMotor.setVelocity(0.0)
    

# Global variable to track turn direction
turnOnBlock = 0
currentTurnDirection = 1 # 0 is for going right, 1 for left

# Main loop: Continue until an exit event occurs
while robot.step(TIME_STEP) != -1:
    # Get sensor readings
    psValues = []
    lsValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
        lsValues.append(ls[i].getValue())

    # Obstacle detection
    front_obstacle = psValues[0] > 150.0 or psValues[7] > 150.0
    right_obstacle = psValues[0] > 80.0 or psValues[1] > 80.0 or psValues[2] > 80.0
    left_obstacle = psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0
    front_light = lsValues[6] == 0.0 and lsValues[7] == 0.0 and lsValues[0] == 0.0

    # Default motor speeds
    leftSpeed = 0.5 * MAX_SPEED
    rightSpeed = 0.5 * MAX_SPEED
    
    #print("Light Sensor Values: ", lsValues)
    
    if turnOnBlock == 1:
        if not front_obstacle and not right_obstacle and not left_obstacle:
            turn_left()
            turnOnBlock = 0
    elif turnOnBlock == 2:
        if not front_obstacle and not right_obstacle and not left_obstacle:
            turn_right()
            turnOnBlock = 0
            
            
    if front_light == True:
        print("Front Light Detected")
        if currentTurnDirection == 0:
            currentTurnDirection = 1
            turn_180()
        elif currentTurnDirection == 1:
            currentTurnDirection = 0
            turn_180()

    # logic for right turns
    if currentTurnDirection == 0:
        
        # Wall-following logic
        if front_obstacle:
            if left_obstacle:
                print("Turning Right")
                turnOnBlock = 1
                turn_right()  # Hardcoded duration
            elif right_obstacle:
                print("Turning Left")
                turnOnBlock = 2
                turn_left()  # Hardcoded duration
        elif left_obstacle:
            leftSpeed = 0.2 * MAX_SPEED  # Slow down left motor to turn right
        elif not left_obstacle and right_obstacle:
            rightSpeed = 0.2 * MAX_SPEED  # Slow down right motor to turn left
            
            
    if currentTurnDirection == 1:
    # Wall-following logic
        if front_obstacle:
            if right_obstacle:
                print("Turning Left")
                turnOnBlock = 2
                turn_left()  # Hardcoded duration
            elif left_obstacle:
                print("Turning Right")
                turnOnBlock = 1
                turn_right()  # Hardcoded duration
        elif left_obstacle:
            leftSpeed = 0.2 * MAX_SPEED  # Slow down left motor to turn right
        elif not left_obstacle and right_obstacle:
            rightSpeed = 0.2 * MAX_SPEED  # Slow down right motor to turn left

    # Set motor velocities
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
