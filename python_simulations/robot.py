# -----------
# User Instructions
#
# Implement a P controller by running 100 iterations
# of robot motion. The steering angle should be set
# by the parameter tau so that:
#
# steering = -tau_p * CTE - tau_d * diff_CTE - tau_i * int_CTE
#
# where the integrated cross track error (int_CTE) is
# the sum of all the previous cross track errors.
# This term works to cancel out steering drift.
#
# Only modify code at the bottom! Look for the TODO.
# ------------

import random
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Cursor
import math

# ------------------------------------------------
#
# this is the Robot class
#
from numpy.core._multiarray_umath import ndarray


class Robot(object):
    def __init__(self, length=20.0):
        """
        Creates robot and initializes location/orientation to 0, 0, 0.
        """
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.length = length
        self.steering_noise = 0.0
        self.distance_noise = 0.0
        self.steering_drift = 0.0

    def set(self, x, y, orientation):
        """
        Sets a robot coordinate.
        """
        self.x = x
        self.y = y
        self.orientation = orientation % (2.0 * np.pi)

    def set_noise(self, steering_noise, distance_noise):
        """
        Sets the noise parameters.
        """
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.steering_noise = steering_noise
        self.distance_noise = distance_noise

    def set_steering_drift(self, drift):
        """
        Sets the systematical steering drift parameter
        """
        self.steering_drift = drift

    def move(self, steering, distance, tolerance=0.001, max_steering_angle=np.pi / 4.0):
        """
        steering = front wheel steering angle, limited by max_steering_angle
        distance = total distance driven, most be non-negative
        """
        if steering > max_steering_angle:
            steering = max_steering_angle
        if steering < -max_steering_angle:
            steering = -max_steering_angle
        if distance < 0.0:
            distance = 0.0

        # apply noise
        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)

        # apply steering drift
        steering2 += self.steering_drift

        # Execute motion
        turn = np.tan(steering2) * distance2 / self.length

        if abs(turn) < tolerance:
            # approximate by straight line motion
            self.x += distance2 * np.cos(self.orientation)
            self.y += distance2 * np.sin(self.orientation)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
        else:
            # approximate bicycle model for motion
            radius = distance2 / turn
            cx = self.x - (np.sin(self.orientation) * radius)
            cy = self.y + (np.cos(self.orientation) * radius)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
            self.x = cx + (np.sin(self.orientation) * radius)
            self.y = cy - (np.cos(self.orientation) * radius)

    def __repr__(self):
        return '[x=%.5f y=%.5f orient=%.5f]' % (self.x, self.y, self.orientation)


# ------ ADD / MODIFY CODE BELOW -----------
# ------------------------------------------------------------------------
#
# run - does a single control run

robot = Robot()
robot.set(0, 1, 0)


def trajectory_pulse(t=50, samples=1000):
    trajectory = []
    toggle = True
    for i in range(samples):
        if i % t == 0:
            if not toggle:
                toggle = 10 * True
            else:
                toggle = False
        trajectory.append(toggle)

    return [i for i in range(n)], trajectory


def trajectory_eight(samples=500):
    t = np.array([i for i in range(samples)])
    r = 1
    p = 60
    xr = -r * np.sin(4 * pi / p * t)
    yr = -2 * r * (1 - np.cos(2 * pi / p * t))

    return xr, yr


def sigmoid(c1, x):
    z = 1 / (1 + np.exp(-c1 * x))
    return z


def fuzzy_model(error, a):

    if error <= (-a):
        y_value = -a
    elif (-a) < error < (-0.5 * a):
        y_value = error
    elif (-0.5 * a) < error < (0.5 * a):
        y_value = 0.5 * error
    elif (0.5 * a) < error < a:
        y_value = error
    else:
        y_value = a

    return y_value / a


def gaussian(x, mu=0, sig=2):
    return math.exp(-math.pow(x - mu, 2.) / (2 * math.pow(sig, 2.)))


def run(robot, tau_p, tau_d, tau_i, n=500, speed=1.0, reference_trajectory=None):
    x_trajectory = []
    y_trajectory = []
    cte = robot.y - reference_trajectory[1][0]
    prev_cte = cte
    sum_cte = 0
    for i in range(n):
        cte = robot.y - reference_trajectory[1][i]
        cte = fuzzy_model(cte, 1)
        #print(cte1, cte)
        #diff_cte = cte-prev_cte
        diff_cte = fuzzy_model(cte-prev_cte, 1)
        prev_cte = cte
        sum_cte += cte
        steer = - tau_p * cte - tau_d * diff_cte
        print('STEERING', steer)
        robot.move(steer, speed)
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)

    return x_trajectory, y_trajectory


n = 1000

reference = trajectory_pulse(200, n)
# reference = trajectory_eight(n)

kp = .1
kd = 10
ki = 0.0001
x_trajectory, y_trajectory = run(robot, kp, kd, ki, n=n, reference_trajectory=reference)

fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, facecolor='#FFFFFF')

ax.plot(x_trajectory, y_trajectory, 'g', label='PID controller')
ax.plot(reference[0], reference[1], 'r', label='Reference')
ax.legend()
ax.annotate(f'KP = {kp}\n KD = {kd}\n KI = {ki}',
            xy=(0, 1), xycoords='data',
            xytext=(0.8, 0.95))
cursor = Cursor(ax, useblit=True, color="b", linewidth=2)
plt.show()
