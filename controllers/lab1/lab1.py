from controller import Robot, DistanceSensor, Motor, LightSensor

# time in [ms] of a simulation step
TIME_STEP = 64

MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# initialize devices
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

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

global turn_status
turn_status = False
global wall_status
wall_status = False


def turn180():
    global turn_status
    turn_status = True
    leftMotor.setVelocity(MAX_SPEED)
    rightMotor.setVelocity(-MAX_SPEED)

    start_time = robot.getTime() * 1000  # convert to ms
    while (robot.getTime() * 1000 - start_time) < 4000:
        robot.step(TIME_STEP)

    # Stop the motors after the turn
    leftMotor.setVelocity(0.0)
    rightMotor.setVelocity(0.0)

    turn_status = False


def follow_left():
    global turn_status
    turn_status = True

    rightMotor.setVelocity(MAX_SPEED * 0.1)
    leftMotor.setVelocity(-MAX_SPEED * 0.1)

    start_time = robot.getTime() * 1000  # convert to ms
    while (robot.getTime() * 1000 - start_time) < 1000:
        robot.step(TIME_STEP)

    # Stop the motors after adjustment
    leftMotor.setVelocity(0.0)
    rightMotor.setVelocity(0.0)

    turn_status = False


def turn_right_90():
    global turn_status
    turn_status = True

    rightMotor.setVelocity(-MAX_SPEED * 0.1)
    leftMotor.setVelocity(MAX_SPEED * 0.1)

    start_time = robot.getTime() * 1000  # convert to ms
    while (robot.getTime() * 1000 - start_time) < 500:
        robot.step(TIME_STEP)

    # Stop the motors after adjustment
    leftMotor.setVelocity(0.0)
    rightMotor.setVelocity(0.0)

    # Move forward a bit to avoid immediate re-detection
    leftMotor.setVelocity(0.5 * MAX_SPEED)
    rightMotor.setVelocity(0.5 * MAX_SPEED)

    start_time = robot.getTime() * 1000  # convert to ms
    while (robot.getTime() * 1000 - start_time) < 500:
        robot.step(TIME_STEP)

    leftMotor.setVelocity(0.0)
    rightMotor.setVelocity(0.0)

    turn_status = False


# feedback loop: step simulation until receiving an exit event
step = robot.step(TIME_STEP)
while robot.step(TIME_STEP) != -1:
    if not turn_status:
        # read sensors outputs
        psValues = []
        lsValues = []
        for i in range(8):
            psValues.append(ps[i].getValue())
            lsValues.append(ls[i].getValue())


        # detect obstacles
        right_obstacle = psValues[1] > 100 or psValues[2] > 100
        left_obstacle = psValues[5] > 100 or psValues[6] > 100
        front_obstacle = psValues[0] > 150 or psValues[7] > 150
        block_left = psValues[4] > 90 and psValues[0] < 150 and psValues[5] < 100

        # initialize motor speeds at 50% of MAX_SPEED.
        leftSpeed = 0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED

        # modify speeds according to obstacles
        if front_obstacle and not wall_status:
            print("Obstacle in Front")
            wall_status = True

        if left_obstacle and not turn_status:
            print("Turning Right")
            turn_right_90()

        elif right_obstacle and not turn_status:
            leftSpeed = -0.5 * MAX_SPEED
            rightSpeed = 0.5 * MAX_SPEED

        if block_left and not turn_status:
            print("Activating Follow Left (Block)")
            follow_left()

        # write actuators inputs
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)

### running code by the numbers:

