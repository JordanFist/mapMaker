from robot import Robot
from Computations import *
from math import pi
from numpy import sign
from random import random
import time


class Controller:
    """
    This class is able to move the robot, given a path to a destination
    """

    def __init__(self, cartographer):
        self.waypoints = []
        # a new destination is computed every time sec
        self.time = 1  # to review
        self.alpha = 0
        self.v = 1
        # lookahead distance
        self.L = 2.5  # to review
        self.offset = 0
        self.DISTANCE_UPDATE = 5
        self.distanceTravelled = 0
        self.LASER_ANGLE = 5
        self.OBSTACLE_MAX_DIST = 2
        self.WANDERING_MIN = int(4 / self.OBSTACLE_MAX_DIST)
        self.WANDERING_MAX = int(10 / self.OBSTACLE_MAX_DIST)

        self.cartographer = cartographer

    def checkObstacle(self, robot):
        """
        If an obstacle is encountered, the robot is stopped
        :return:
        """
        lasers = robot.getLaser()
        for i in range(len(lasers['Echoes']) // 2 - self.LASER_ANGLE,
                       len(lasers['Echoes']) // 2 + self.LASER_ANGLE):
            if lasers['Echoes'][i] < self.OBSTACLE_MAX_DIST:
                robot.setMotion(0, 0)
                return True
        return False
        
    def moveOnce(self, robot, destination):
        """
        Move the robot to a new destination
        :param robot: Robot object
        :param destination: quaternion
        """
        r = getRadius(robot.getPosition(), destination, robot.getHeading())
        robot.setMotion(self.v, sign(self.alpha) * self.v / r)
        time.sleep(self.time)

    def rad2deg(self, alpha):
        return 180 * alpha / pi

    def getNextPoint(self, robot, path):
        """
        Computes the next destination in the path
        :param robot: Robot object
        :param path: list of quaternions
        :return: the destination (quaternion)
        """
        A, B = {}, {}
        for i in range(self.offset, len(path)):
            d = getDistance(robot.getPosition(), path[i])
            if (d >= self.L) and i > self.offset:
                self.offset = i - 1
                try:
                    A, B = getLineCircleIntersection(robot.getPosition(), self.L, path[i], path[i - 1])
                except:
                    print(robot.getPosition(), self.L, path[i], path[i - 1])
                    print(path)
                    print(self.offset)
                    print(self.waypoints)
                    return None
            elif (d >= self.L) and i == self.offset:
                self.offset = 0
                A, B = getLineCircleIntersection(robot.getPosition(), self.L, path[i], robot.getPosition())
            if (A, B) != ({}, {}):
                if getDistance(A, path[i]) < getDistance(B, path[i]):
                    self.waypoints.append(A)
                    return A
                else:
                    self.waypoints.append(B)
                    return B
        return path[-1]
      
    def move(self, robot, path):
        """
        Moves the robot following a given path
        :param robot: Robot object
        :param path: list of quaternion
        :return False iff something went wrong (an obstacle was on the way)
        """
        self.waypoints = []
        self.offset = 0
        nextPoint = self.getNextPoint(robot, path)
        while (nextPoint != path[-1]):
            nextPoint = self.getNextPoint(robot, path)
            self.alpha = getAlpha(robot.getPosition(), nextPoint, robot.getHeading())
            self.moveOnce(robot, nextPoint)
            if self.checkObstacle(robot):
                return False
            self.checkDistanceTravelled(robot)
        self.moveOnce(robot, nextPoint)
        robot.setMotion(0, 0)
        return True

    def checkDistanceTravelled(self, robot):
        self.distanceTravelled += self.L
        if self.distanceTravelled >= self.DISTANCE_UPDATE:
            self.cartographer.update(robot)
            self.distanceTravelled = 0

    def wander(self, robot):
        run = True
        while run:
            robot.setMotion(0, 2 * pi * random())
            time.sleep(1)
            if not self.checkObstacle(robot):
                run = False
                for _ in range(self.WANDERING_MIN,
                               int(random() * (self.WANDERING_MAX - self.WANDERING_MIN) + self.WANDERING_MIN)):
                    robot.setMotion(self.OBSTACLE_MAX_DIST, 0)
                    time.sleep(1)
                    self.cartographer.update(robot)
                    if self.checkObstacle(robot):
                        run = True
                        break
