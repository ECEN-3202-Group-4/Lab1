import signal
import atexit
import numpy as np
from matplotlib import pyplot as plt
import math
from controller import Robot, DistanceSensor, Motor, LightSensor
from enum import Enum, auto

# Simulation time step in milliseconds
TIME_STEP_MS = 64
TIME_STEP = TIME_STEP_MS / 1000

# General Behavoir
WHEEL_RADIUS = 0.02
MAX_SPEED = WHEEL_RADIUS * 6.28  # max speed in m/s
STANDARD_SPEED = MAX_SPEED / 2

# Turning behavior
TURN_MAX = 90
TURN_MIN = -90
MAX_TURN_SPEED = STANDARD_SPEED

light_sensor_values = []
light_sensor_count = []
highlight = 0


class RobotState(Enum):
    LEFT_WALL = auto()
    RIGHT_WALL = auto()
    STOP = auto()


class LightSensor:
    def __init__(self, name, r):
        self.name = name
        self.sensor = r.getDevice(name)
        table = self.sensor.getLookupTable()
        self.lookup_table = [
            table[i:i + 3] for i in range(0, len(table), 3)
        ]
        self.irradiances = [row[0] for row in self.lookup_table]  # Input irradiance values
        self.sensor_outputs = [row[1] for row in self.lookup_table]  # Sensor output values
        self.sensor.enable(TIME_STEP_MS)

    def calculate_irradiance(self, sensor_value):
        if sensor_value >= self.sensor_outputs[0]:
            return self.irradiances[0]
        elif sensor_value <= self.sensor_outputs[-1]:
            return self.irradiances[-1]

        # Linear interpolation between lookup table points
        for i in range(len(self.sensor_outputs) - 1):
            if self.sensor_outputs[i] >= sensor_value >= self.sensor_outputs[i + 1]:
                # Interpolate correctly
                y0, x0 = self.sensor_outputs[i], self.irradiances[i]
                y1, x1 = self.sensor_outputs[i + 1], self.irradiances[i + 1]
                return x0 + (sensor_value - y0) * (x1 - x0) / (y1 - y0)
        # Return None if no match (shouldn't happen)
        return None

    def read_irradiance(self):
        sensor_value = self.sensor.getValue()
        irradiance = self.calculate_irradiance(sensor_value)
        # Optionally, add noise based on noise_levels
        return irradiance


class DistanceSensor:
    def __init__(self, name, r):
        self.name = name
        self.sensor = r.getDevice(name)
        angles = {
            "ps0": 17.234570,
            "ps1": 45.882460,
            "ps2": 90.000153,
            "ps3": 151.489239,
            "ps4": 208.784979,
            "ps5": 270.000362,
            "ps6": 314.209213,
            "ps7": 342.857103
        }
        self.angle = angles.get(name, 0)
        table = self.sensor.getLookupTable()
        self.lookup_table = [
            table[i:i + 3] for i in range(0, len(table), 3)
        ]
        self.distances = [row[0] for row in self.lookup_table]  # Input distances
        self.responses = [row[1] for row in self.lookup_table]  # Response values
        self.sensor.enable(TIME_STEP_MS)

    def calculate_distance(self, sensor_value):
        if sensor_value <= self.responses[-1]:
            return self.distances[-1]
        elif sensor_value >= self.responses[0]:
            return self.distances[0]

        # Linear interpolation between lookup table points
        for i in range(len(self.responses) - 1):
            if self.responses[i] >= sensor_value >= self.responses[i + 1]:
                # Interpolate
                x0, y0 = self.responses[i], self.distances[i]
                x1, y1 = self.responses[i + 1], self.distances[i + 1]
                return y0 + (sensor_value - x0) * (y1 - y0) / (x1 - x0)
        # Return None if no match (shouldn't happen)
        return None

    def read(self):
        return self.calculate_distance(self.sensor.getValue())


class WallFollower:
    def __init__(self, r):
        self.prev_distances = []
        self.DIST_SIZE = 5
        self.DESIRED_DIST = 3
        self.WALL_FOLLOW_KP = 0.2
        self.WALL_FOLLOW_KI = 0
        self.WALL_FOLLOW_KD = 0.1
        self.integral = 0.0
        self.prev_error = -np.inf
        self.robot = r

    def clear(self):
        self.prev_distances = []
        self.integral = 0.0
        self.prev_error = -np.inf

    def execute(self):
        dist: float = -1
        if self.robot.state == RobotState.LEFT_WALL:
            dist = self.robot.get_distance('left')
        elif self.robot.state == RobotState.RIGHT_WALL:
            dist = self.robot.get_distance('right')
        if dist == -1:
            raise ValueError("Distance sensor read error.")

        dist = 100 * dist  # convert from m to mm
        if dist >= 6.8:  # can't see wall
            self.clear()
            covered_distance = 0.0
            while covered_distance < 0.04:
                self.robot.go_forward(STANDARD_SPEED)
                covered_distance += STANDARD_SPEED * TIME_STEP
                self.robot.robot.step(TIME_STEP_MS)

            if self.robot.state == RobotState.LEFT_WALL:
                angle = self.robot.angle - 90
                while self.robot.angle > angle:
                    self.robot.turn(0, -90)
                    self.robot.robot.step(TIME_STEP_MS)
            elif self.robot.state == RobotState.RIGHT_WALL:
                angle = self.robot.angle + 90
                while self.robot.angle < angle:
                    self.robot.turn(0, 90)
                    self.robot.robot.step(TIME_STEP_MS)
            covered_distance = 0.0
            while covered_distance < 0.06:
                self.robot.go_forward(STANDARD_SPEED)
                covered_distance += STANDARD_SPEED * TIME_STEP
                self.robot.robot.step(TIME_STEP_MS)
            return
        self.prev_distances.append(dist)
        if len(self.prev_distances) > self.DIST_SIZE:
            self.prev_distances.pop(0)
        avg_dist = np.mean(self.prev_distances)
        error = self.DESIRED_DIST - avg_dist

        # PID
        self.integral += error * TIME_STEP
        if self.prev_error != -np.inf:
            derivative = (error - self.prev_error) / TIME_STEP
        else:
            derivative = 0
        self.prev_error = error

        w = (self.WALL_FOLLOW_KP * error) + (self.WALL_FOLLOW_KI * self.integral) + (
                self.WALL_FOLLOW_KD * derivative)
        w_deg = np.degrees(w)
        if self.robot.state is RobotState.RIGHT_WALL:
            w_deg *= -1
        if abs(w_deg) > 2:
            self.robot.turn(MAX_TURN_SPEED, w_deg)
        else:
            self.robot.turn(2 * (MAX_SPEED / 3), w_deg)


class RobotClass:
    def __init__(self):
        self.state = RobotState.LEFT_WALL
        self.robot = Robot()
        self.ps: dict[str, DistanceSensor] = {}
        ps_names = ['ps0', 'ps1', 'ps2', 'ps3',
                    'ps4', 'ps5', 'ps6', 'ps7']
        self.ls: dict[str, LightSensor] = {}
        ls_names = ['ls0', 'ls1', 'ls2', 'ls3',
                    'ls4', 'ls5', 'ls6', 'ls7']
        for name in ps_names:
            self.ps[name] = DistanceSensor(name, self.robot)

        for name in ls_names:
            self.ls[name] = LightSensor(name, self.robot)

        self.motors = {'left': self.robot.getDevice('left wheel motor'),
                       'right': self.robot.getDevice('right wheel motor')}
        self.motors['left'].setPosition(float('inf'))
        self.motors['right'].setPosition(float('inf'))
        self.motors['left'].setVelocity(0.0)
        self.motors['right'].setVelocity(0.0)
        self.angle = 0
        self.wall_follower = WallFollower(self)

    def turn(self, v, w):
        # turns the robot at w (deg/sec). + for right, - for left
        # while maintain a forward velocity of v (m/s)
        w = np.clip(w, TURN_MIN, TURN_MAX)
        self.angle += (w * TIME_STEP)
        # convert to radians
        w_rad = np.radians(w)
        wheelbase = 0.052  # from the e-puck proto file

        v_rel = w_rad * (wheelbase / 2)  # relative velocity

        v_left = v + v_rel
        v_right = v - v_rel  # Adjust speeds to ensure they don't exceed max wheel speed
        while abs(v_left) > MAX_SPEED or abs(v_right) > MAX_SPEED:
            # Scale down v to ensure both wheel speeds stay within limits
            scaling_factor = max(abs(v_left) / MAX_SPEED, abs(v_right) / MAX_SPEED)
            v /= scaling_factor
            # Recalculate v_left and v_right with the scaled v
            v_left = v + v_rel
            v_right = v - v_rel

        # convert to w
        w_left = v_left / WHEEL_RADIUS
        w_right = v_right / WHEEL_RADIUS

        self.motors['left'].setVelocity(w_left)
        self.motors['right'].setVelocity(w_right)

    def go_forward(self, v):
        w = v / WHEEL_RADIUS
        self.motors['left'].setVelocity(w)
        self.motors['right'].setVelocity(w)

    def get_distance(self, direction):
        if direction == 'forward':
            name: str
            if self.state == RobotState.LEFT_WALL:
                name = 'ps0'
            elif self.state == RobotState.RIGHT_WALL:
                name = 'ps7'
            else:
                return -1
            h = self.ps[name].read()
            angle = self.ps[name].angle
            return h * math.cos(np.radians(angle))


        elif direction == 'backward':
            name: str
            if self.state == RobotState.LEFT_WALL:
                name = 'ps3'
            elif self.state == RobotState.RIGHT_WALL:
                name = 'ps4'
            else:
                return -1
            h = self.ps[name].read()
            angle = self.ps[name].angle
            return h * math.cos(np.radians(angle))
        elif direction == 'left':
            return self.ps['ps5'].read()
        elif direction == 'right':
            return self.ps['ps2'].read()

    def evaluate_state(self):
        if self.state == RobotState.STOP:
            return  # do nothing
        local_irradiance_values = []
        sensors = ['ls0', 'ls1', 'ls2', 'ls5', 'ls6', 'ls7']
        for sensor in sensors:
            irradiance = self.ls[sensor].read_irradiance()
            local_irradiance_values.append(irradiance)  # Store in local list
            light_sensor_values.append(irradiance)  # Append to global list

        threshold = 2
        count_above_threshold = sum(value >= threshold for value in local_irradiance_values)
        light_sensor_count.append(count_above_threshold)
        if count_above_threshold == len(local_irradiance_values):
            if self.state == RobotState.LEFT_WALL:
                # turn 180
                angle = self.angle - 180
                self.turn(0, -90)
                self.robot.step(TIME_STEP_MS)
                while self.angle > angle:
                    self.turn(0, -90)
                    self.robot.step(TIME_STEP_MS)

                self.wall_follower.clear()
                self.state = RobotState.RIGHT_WALL
            elif self.state == RobotState.RIGHT_WALL:
                self.wall_follower.clear()
                self.go_forward(0)
                self.state = RobotState.STOP
        # TODO

    def execute_state(self):
        # TODO

        if self.state == RobotState.STOP:
            return  # do nothing
        if self.state == RobotState.LEFT_WALL or self.state == RobotState.RIGHT_WALL:
            forward_dist = self.get_distance('forward')
            forward_dist *= 100
            if forward_dist > self.wall_follower.DESIRED_DIST:
                self.wall_follower.execute()
            else:
                self.wall_follower.clear()
                if self.state == RobotState.LEFT_WALL:
                    angle = self.angle + 90
                    while self.angle < angle:
                        self.turn(0, 90)
                        self.robot.step(TIME_STEP_MS)
                elif self.state == RobotState.RIGHT_WALL:
                    angle = self.angle - 90
                    while self.angle > angle:
                        self.turn(0, -90)
                        self.robot.step(TIME_STEP_MS)


# Global variable to track turn direction
robot = RobotClass()

import matplotlib.pyplot as plt
import numpy as np


def histogram(data, bins=20):
    plt.ion()  # Enable interactive mode
    plt.figure()

    # Calculate histogram data
    hist, bin_edges = np.histogram(data, bins=bins)
    total = sum(hist)  # Total count of all data points

    # Calculate widths of bars and bin centers
    width = 0.7 * (bin_edges[1] - bin_edges[0])
    center = (bin_edges[:-1] + bin_edges[1:]) / 2

    # Convert counts to percentages
    percentages = (hist / total) * 100

    # Plot the histogram as a bar chart
    bars = plt.bar(center, hist, align='center', width=width)

    # Label the percentages above the bars
    for bar, percentage in zip(bars, percentages):
        plt.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 1,
                 f'{percentage:.1f}%', ha='center', fontsize=8)

    # Customize plot labels and title
    plt.xlabel("Bins")
    plt.ylabel("Counts")
    plt.title("Histogram with Percentage Occurrences")
    plt.show()

    return hist, bin_edges


def handle_sigint(signum, frame):
    histogram(light_sensor_values, 20)


def cleanup():
    print("Program exiting. Generating histogram...")
    histogram(light_sensor_values, 20)


# Register both the signal handler and the atexit cleanup
signal.signal(signal.SIGINT, handle_sigint)
atexit.register(cleanup)

# Main loop: Continue until an exit event occurs
while robot.robot.step(TIME_STEP_MS) != -1:
    # Set motor velocities
    robot.evaluate_state()
    robot.execute_state()
