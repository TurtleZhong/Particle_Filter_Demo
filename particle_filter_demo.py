"""
This project is an implement of particle filter.
* Author: ZhongXinliang
* Email: xinliangzhong@foxmail.com
* Date: 2018.01.17
"""
from math import *
import random
import cv2
import numpy as np

landmarks = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]
world_size = 100.0


class Robot:
    """
    This class aims for describing a robot.
    """
    def __init__(self):
        self.x = random.random() * world_size
        self.y = random.random() * world_size
        self.orientation = random.random() * 2.0 * pi
        self.forward_noise = 0.0
        self.turn_noise = 0.0
        self.sense_noise = 0.0

    def set(self, new_x, new_y, new_orientation):
        """
        This function aims to set the 2d pose of the robot.
        :param new_x: The x coordinate of the robot.
        :param new_y: The y coordinate of the robot.
        :param new_orientation: The orientation of the robot.
        :return: None.
        """
        if new_x < 0 or new_x >= world_size:
            raise ValueError, 'X coordinate out of bound'
        if new_y < 0 or new_y >= world_size:
            raise ValueError, 'Y coordinate out of bound'
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError, 'Orientation must be in [0..2pi]'
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)

    def set_noise(self, new_f_noise, new_t_noise, new_s_noise):
        """
        This function makes it possible to change the noise parameters and
        it is often useful in particle filters.
        :param new_f_noise: The forward noise.
        :param new_t_noise: The turn noise.
        :param new_s_noise: The observation noise.
        :return: None
        """
        self.forward_noise = float(new_f_noise)
        self.turn_noise = float(new_t_noise)
        self.sense_noise = float(new_s_noise)

    def sense(self):
        """
        This function aims to get the observation of the robot.
        :return: A list of the observation. The size depend on the landmarks.
        """
        Z = []
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, self.sense_noise)
            Z.append(dist)
        return Z

    def move(self, turn, forward):
        """
        This function realize the movement of the robot.
        :param turn: The orientation of the robot.
        :param forward: The length of the robot need to move.
        :return: The robot which is moved according to the input params.
        """
        if forward < 0:
            raise ValueError, 'Robot cant move backwards'
        # turn, and add randomness to the turning command
        orientation = self.orientation + float(turn) + random.gauss(0.0, self.turn_noise)
        orientation %= 2 * pi

        # move, and add randomness to the motion command
        dist = float(forward) + random.gauss(0.0, self.forward_noise)
        x = self.x + (cos(orientation) * dist)
        y = self.y + (sin(orientation) * dist)
        x %= world_size  # cyclic truncate
        y %= world_size

        # set particle
        res = Robot()
        res.set(x, y, orientation)
        res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)
        return res

    def gaussian(self, mu, sigma, x):
        """
        This function aims to calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        :param mu: The mean of the gaussian distribution.
        :param sigma: The variance of the gaussian distribution.
        :param x: The input value.
        :return: The probability of the input x.
        """
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))

    def measurement_prob(self, measurement):
        """
        This function aims to calculates how likely a measurement should be.
        :param measurement: The observation of the robot or particle.
        :return: The probability of the measurement.
        """
        prob = 1.0
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            prob *= self.gaussian(dist, self.sense_noise, measurement[i])
        return prob

    def __repr__(self):
        """
        This function aims to overload the print.
        :return: The position of the robot.
        """
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))


def create_map():
    """
    This function aims to create the map.
    :return: An image of the map.
    """
    map_image = np.zeros(shape=(500, 500, 3), dtype=np.uint8)
    for landmark in landmarks:
        cv2.circle(map_image, tuple((5 * int(landmark[0]), 5 * int(landmark[1]))), radius=5, color=(0, 0, 255), thickness=-1)
    return map_image


def show_robot_pose(robot, map_image):
    """
    This function aims to show the robot pose with blue point.
    :param robot: Robot to show.
    :param map_image: Show in which image.
    :return: The image with the robot pose.
    """
    cv2.circle(map_image, tuple((int(5 * robot.x), int(5 * robot.y))), radius=5, color=(255, 0, 0), thickness=-1)
    return map_image


def show_particles(particles, map_image):
    """
    This function aims to show the particles.
    :param particles: Particles to show
    :param map_image: Show in which image.
    :return: The image with the particles.
    """
    map_image_copy = map_image.copy()
    for particle in particles:
        cv2.circle(map_image_copy, tuple((int(5 * particle.x), int(5 * particle.y))),
                   radius=1, color=(0, 255, 0), thickness=-1)
    return map_image_copy

# Step1: Show the map.
map_image = create_map()
cv2.namedWindow('map_image')
cv2.imshow('map_image', map_image)
# Create the robot.
myrobot = Robot()
# Show the robot pose in the map.
map_image = show_robot_pose(myrobot, map_image)
cv2.imshow('map_image', map_image)
cv2.waitKey(0)
# Generate particles.
N = 1000
p = []
for i in range(N):
    x = Robot()
    x.set_noise(0.05, 0.05, 5)
    p.append(x)

initial_image = show_particles(p, map_image.copy())
cv2.imshow('map_image', initial_image)
cv2.waitKey(0)

for i in range(60):
    myrobot = myrobot.move(0.1, 5.0)
    cv2.imshow('map_image', map_image)
    cv2.waitKey(300)
    map_image = show_robot_pose(myrobot, map_image)
    Z = myrobot.sense()
    p2 = []
    for i in range(N):
        p2.append(p[i].move(0.1, 5.0))
    p = p2
    w = []
    for i in range(N):
        w.append(p[i].measurement_prob(Z))
    # Re-sampling
    p3 = []
    index = int(random.random() * N)
    beta = 0.0
    mw = max(w)
    for i in range(N):
        beta += random.random() * 2.0 * mw
        while beta > w[index]:
            beta -= w[index]
            index = (index + 1) % N
        p3.append(p[index])
    p = p3
    initial_image = show_particles(p, map_image.copy())
    cv2.imshow('map_image', initial_image)
    cv2.waitKey(300)
print p

initial_image = show_particles(p, map_image.copy())
cv2.imshow('map_image', initial_image)
initial_image = show_robot_pose(myrobot, initial_image)
cv2.imshow('map_image', initial_image)
cv2.waitKey(0)
