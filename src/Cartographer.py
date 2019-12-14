from show_map import ShowMap
import numpy as np
from math import floor, cos, sin
from Computations import pathToObstacle, getDistance


class Cartographer:

    def __init__(self, xMin, xMax, yMin, yMax, showGUI):
        self.CELL_SIZE = 1
        self.EMPTY = 0
        self.OCCUPIED = 1
        self.UNKNOWN = -1
        self.MAXVALUE = 15
        self.MINVALUE = 0
        self.EMPTY_THRESHOLD = 6
        self.OCCUPIED_THRESHOLD = 8
        self.LASER_MAX = 10

        self.xMin = xMin
        self.xMax = xMax
        self.yMin = yMin
        self.yMax = yMax
        self.showMap = ShowMap(self.getHeight(), self.getWidth(), showGUI)
        self.map = np.ones((self.getHeight(), self.getWidth())) * (self.MAXVALUE + self.MINVALUE) // 2

    def getHeight(self):
        """
        :return: the number of squares in terms of height
        """
        return (self.yMax - self.yMin) // self.CELL_SIZE

    def getWidth(self):
        """
        :return: the number of squares in terms of width
        """
        return (self.xMax - self.xMin) // self.CELL_SIZE

    def isOutOfBound(self, square):
        return not (0 <= square[0] < self.getHeight() and 0 <= square[1] < self.getWidth())

    def getState(self, square):
        if self.map[square[0]][square[1]] < self.EMPTY_THRESHOLD:
            return self.EMPTY
        elif self.map[square[0]][square[1]] > self.OCCUPIED_THRESHOLD:
            return self.OCCUPIED
        else:
            return self.UNKNOWN

    def update(self, robot):
        self.handleLasers(robot)
        row, col = self.getGridPosition(robot.getPosition())
        self.showMap.updateMap(self.map, self.MAXVALUE, row, col)

    def handleLasers(self, robot):
        robotPosition = robot.getPosition()
        lasers = robot.getLaser()
        laserAngles = robot.getAngles()
        obstaclePos = {}
        for i in range(len(lasers['Echoes'])):
            obstaclePos['X'] = lasers['Echoes'][i] * cos(laserAngles[i]) + robotPosition['X']
            obstaclePos['Y'] = lasers['Echoes'][i] * sin(laserAngles[i]) + robotPosition['Y']
            path = pathToObstacle(self.getGridPosition(robotPosition), self.getGridPosition(obstaclePos))
            self.HIMMUpdate(path, robot)

    def HIMMUpdate(self, path, robot):
        for (x, y) in path:
            realPos = self.getRealPosition((x, y))
            distanceToRobot = getDistance(robot.getPosition(), realPos)
            if not self.isOutOfBound((x, y)) and distanceToRobot < self.LASER_MAX:
                if (x, y) == path[-1]:
                    self.map[x][y] = min(self.MAXVALUE, self.map[x][y] + 3)
                else:
                    self.map[x][y] = max(self.MINVALUE, self.map[x][y] - 1)

    def getMap(self):
        return self.map

    def getGridPosition(self, coord):
        col = floor((coord['X'] - self.xMin) / self.CELL_SIZE)
        row = floor((coord['Y'] - self.yMin) / self.CELL_SIZE)
        return (row, col)

    def getRealPosition(self, square):
        x = (square[0] * self.CELL_SIZE + self.xMin) + self.CELL_SIZE / 2
        y = (square[1] * self.CELL_SIZE + self.yMin) + self.CELL_SIZE / 2
        return {'X': x, 'Y': y}

    def getNextLayer(self, layer, inside):
        nextLayer = []
        for square in layer:
            for (i, j) in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    neighbor = (square[0] + i, square[1] + j)
                    if not self.isOutOfBound(neighbor) and neighbor not in inside and not self.getState(neighbor) == self.OCCUPIED:
                        nextLayer.append((neighborRow, neighborCol))
        return nextLayer

    def isOnBorder(self, square):
        if self.getState(square) == self.UNKNOWN:
            return False
        for (i, j) in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                neighbor = (square[0] + i, square[1] + j)
                if self.getState(neighbor) == self.UNKNOWN and not self.isOutOfBound(neighbor):
                    return True
        return False

    def findBorderSquare(self, square, acc):
        if self.isOnBorder(square):
            return square
        acc.add(square)
        for i in range(-1, 2):
            for j in range(-1, 2):
                neighbor = (square[0] + i, square[1] + j)
                if not neighbor in acc and not self.isOutOfBound(neighbor):
                    borderSquare = self.findBorderSquare(neighbor, acc)
                    if borderSquare:
                        return borderSquare
        return None

    def findBorder(self, square):
        border = []
        ends = []
        stack = [square]
        while len(stack) != 0:
            border.append(stack.pop())
            for i in range(-1, 2):
                for j in range(-1, 2):
                    neighbor = (square[0] + i, square[1] + j)
                    if self.isOnBorder(neighbor) and not neighbor in border:
                        if self.getState(neighbor) == self.getState(square):
                            stack.append(neighbor)
                        else:
                            ends.append(neighbor)
        return border, ends








