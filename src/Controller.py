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
        # a new destination is computed every time sec
        self.time = 1
        self.alpha = 0
        self.v = 1
        # lookahead distance
        self.L = 4
        self.offset = 0
        self.DISTANCE_UPDATE = 4
        self.distanceTravelled = 0
        self.LASER_ANGLE = 8
        self.OBSTACLE_MAX_DIST = 2
        self.WANDERING_DISTANCE = 8

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
        robot.setMotion(0, 0)

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
                A, B = getLineCircleIntersection(robot.getPosition(), self.L, path[i], path[i - 1])
            elif (d >= self.L) and i == self.offset:
                A, B = getLineCircleIntersection(robot.getPosition(), self.L, path[i], robot.getPosition())
            if (A, B) != ({}, {}):
                if getDistance(A, path[i]) < getDistance(B, path[i]):
                    return A
                else:
                    return B
        return path[-1]

    def orientToward(self, robot, pos):
        angle = getAlpha(robot.getPosition(), pos, robot.getHeading()) / 2
        robot.setMotion(0, angle)
        time.sleep(1)

    def move(self, robot, path):
        """
        Moves the robot following a given path
        :param robot: Robot object
        :param path: list of quaternion
        :return False iff something went wrong (an obstacle was on the way)
        """
        self.orientToward(robot, path[0])
        self.offset = 0
        nextPoint = self.getNextPoint(robot, path)
        while (nextPoint != path[-1]):
            nextPoint = self.getNextPoint(robot, path)
            self.alpha = getAlpha(robot.getPosition(), nextPoint, robot.getHeading())
            self.moveOnce(robot, nextPoint)
            if self.checkObstacle(robot):
                return False
            self.checkDistanceTravelled(robot, self.time * self.v)
        self.moveOnce(robot, nextPoint)
        return True

    def checkDistanceTravelled(self, robot, dist):
        """
        Called each time a new destination is computed, this function updates the map if the robot has moved enough
        :param robot: Robot object
        """
        self.distanceTravelled += dist
        if self.distanceTravelled >= self.DISTANCE_UPDATE:
            self.cartographer.update(robot)
            self.distanceTravelled = 0

    def wander(self, robot):
        """
        Random behaviour, avoiding obstacles
        :param robot: Robot object
        """
        run = True
        while run:
            robot.setMotion(0, 2 * pi * random())
            time.sleep(1)
            if not self.checkObstacle(robot):
                run = False
                for _ in range(self.WANDERING_DISTANCE):
                    robot.setMotion(self.OBSTACLE_MAX_DIST * 2, 0)
                    time.sleep(0.5)
                    robot.setMotion(0, 0)
                    self.checkDistanceTravelled(robot, self.OBSTACLE_MAX_DIST)
                    robotPos = self.cartographer.getGridPosition(robot.getPosition())
                    if self.checkObstacle(robot) or self.cartographer.isOutOfBound(robotPos):
                        break
