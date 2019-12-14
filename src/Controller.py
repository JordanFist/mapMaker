from robot import Robot
from Computations import *
from math import pi
from numpy import sign


class Controller:
    """
    This class is able to move the robot, given a path to a destination
    """

    def __init__(self, cartographer):
        # a new destination is computed every time sec
        self.time = 1  # to review
        self.alpha = 0
        self.v = 0.7
        # lookahead distance
        self.L = 2  # to review
        self.offset = 0

        self.cartographer = cartographer

    def isObstacle(self):
        """
        If an obstacle is encountered, the robot is stopped
        :return:
        """
        pass
        
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
        for i in range(offset, len(path)):
            d = getDistance(robot.getPosition(), path[i])
            if (d >= self.L):
                self.offset = i
                A, B = getLineCircleIntersection(robot.getPosition(), self.L, path[i], path[i - 1])
                if getDistance(A, path[i]) < getDistance(B, path[i]):
                    return A
                else:
                    return B
        return path[-1]
      
    def move(self, robot, path):
        """
        Moves the robot following a given path
        :param robot: Robot object
        :param path: list of quaternion
        """
        nextPoint = self.getNextPoint(robot, path)
        while (nextPoint != path[-1]):
            nextPoint = self.getNextPoint(robot, path)
            self.alpha = getAlpha(robot.getPosition(), nextPoint, robot.getHeading())
            self.moveOnce(robot, nextPoint)
        self.moveOnce(robot, nextPoint)
        robot.setMotion(0, 0)
