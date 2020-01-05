from show_map import ShowMap
import numpy as np
from math import floor, cos, sin, atan2
from Computations import pathToObstacle, getDistance


class Cartographer:
    """
    This class stores and updates the internal representation of the environment
    and serves as an interface to get information on the map
    """

    def __init__(self, xMin, xMax, yMin, yMax, showGUI):
        self.CELL_SIZE = 0.3
        self.EMPTY = 0
        self.OCCUPIED = 1
        self.UNKNOWN = -1
        self.MAXVALUE = 15
        self.MINVALUE = 0
        self.EMPTY_THRESHOLD = 7
        self.OCCUPIED_THRESHOLD = 7
        self.LASER_MAX_DISTANCE = 10
        self.LASER_MAX_ANGLE = 50
        self.ROBOT_WIDTH = 0.8

        self.xMin = xMin
        self.xMax = xMax
        self.yMin = yMin
        self.yMax = yMax
        self.showMap = ShowMap(self.getWidth(), self.getHeight(), showGUI)
        self.map = np.ones((self.getWidth(), self.getHeight())) * (self.MAXVALUE + self.MINVALUE) // 2

    def getHeight(self):
        """
        :return: the number of squares in terms of height
        """
        return int((self.yMax - self.yMin) / self.CELL_SIZE)

    def getWidth(self):
        """
        :return: the number of squares in terms of width
        """
        return int((self.xMax - self.xMin) / self.CELL_SIZE)

    def isOutOfBound(self, square):
        """
        :param square: a pair
        :return: True iff the given square is outside the grid's limits
        """
        return not (0 <= square[0] < self.getWidth() and 0 <= square[1] < self.getHeight())

    def getState(self, square):
        """
        Given a square, returns one of these states : EMPTY, OCCUPIED or UNKNOWN
        :param square: a pair
        :return: the state of the square
        """
        if self.map[square[0]][square[1]] < self.EMPTY_THRESHOLD:
            return self.EMPTY
        elif self.map[square[0]][square[1]] > self.OCCUPIED_THRESHOLD:
            return self.OCCUPIED
        else:
            return self.UNKNOWN

    def getSquaresInCircle(self, center, radius):
        squares = set()
        toProcess = [self.getGridPosition(center)]
        while len(toProcess) > 0:
            square = toProcess.pop()
            pos = self.getRealPosition(square)
            if getDistance(pos, center) < radius:
                squares.add(square)
                for i in range(-1, 2):
                    for j in range(-1, 2):
                        neighbor = square[0] + i, square[1] + j
                        if neighbor not in squares and not self.isOutOfBound(neighbor):
                            toProcess.append(neighbor)
        return squares

    def increaseThickness(self, robot):
        squares = self.getSquaresInCircle(robot.getPosition(), self.LASER_MAX_DISTANCE)
        while len(squares) > 0:
            square = squares.pop()
            if self.getState(square) == self.OCCUPIED:
                neighbors = self.getSquaresInCircle(self.getRealPosition(square), self.ROBOT_WIDTH)
                squares -= neighbors
                for neighbor in neighbors:
                    if self.getState(neighbor) == self.OCCUPIED:
                        path = pathToObstacle(square, neighbor)
                        for (row, col) in path:
                            self.map[row][col] = min(self.MAXVALUE, self.map[row][col] + 3)

    def update(self, robot):
        """
        Updates the map representation
        :param robot: Robot object
        """
        if self.isOutOfBound(self.getGridPosition(robot.getPosition())):
            return
        self.handleLasers(robot)
        self.increaseThickness(robot)
        row, col = self.getGridPosition(robot.getPosition())
        self.showMap.updateMap(self.map, self.MAXVALUE, row, col)

    def handleLasers(self, robot):
        """
        Uses the laser scanners to detect obstacles and updates the map accordingly
        :param robot: Robot object
        """
        robotPosition = robot.getPosition()
        robotHeading = robot.getHeading()
        robotAngle = atan2(robotHeading["Y"], robotHeading["X"])
        lasers = robot.getLaser()
        laserAngles = robot.getAngles()
        obstaclePos = {}
        # Find a potential obstacle for each laser
        for i in range(len(lasers['Echoes']) // 2 - self.LASER_MAX_ANGLE,
                       len(lasers['Echoes']) // 2 + self.LASER_MAX_ANGLE):
            obstaclePos['X'] = lasers['Echoes'][i] * cos(laserAngles[i] + robotAngle) + robotPosition['X']
            obstaclePos['Y'] = lasers['Echoes'][i] * sin(laserAngles[i] + robotAngle) + robotPosition['Y']
            path = pathToObstacle(self.getGridPosition(robotPosition), self.getGridPosition(obstaclePos))
            self.HIMMUpdate(path, robotPosition)

    def HIMMUpdate(self, path, robotPosition):
        """
        Given a path from the robot to a potential obstacle, updates the map using HIMM method
        :param path: a list of pairs, starting from the robot's position
        :param robot: Robot object
        """
        GROMask = [[0.5, 0.5, 0.5], [0.5, 1, 0.5], [0.5, 0.5, 0.5]]
        for (x, y) in path:
            realPos = self.getRealPosition((x, y))
            distanceToRobot = getDistance(robotPosition, realPos)
            # Do not update beyond the LASER_MAX distance
            if distanceToRobot > self.LASER_MAX_DISTANCE:
                break
            if not self.isOutOfBound((x, y)):
                if (x, y) == path[-1]:
                    self.map[x][y] = min(self.MAXVALUE, self.map[x][y] + 3)
                    # Computation of the growth operator
                    sum = 0
                    for i in range(-1, 2):
                        for j in range(-1, 2):
                            neighborRow, neighborCol = x + i, y + j
                            if not self.isOutOfBound((neighborRow, neighborCol)):
                                sum += self.map[neighborRow][neighborCol] * GROMask[1 + i][1 + j]
                                # Increase thickness of obstacles
                                # self.map[neighborRow][neighborCol] = min(self.MAXVALUE, self.map[neighborRow][neighborCol] + 3)
                    self.map[x][y] = min(self.MAXVALUE, sum)
                else:
                    self.map[x][y] = max(self.MINVALUE, self.map[x][y] - 1)

    def getMap(self):
        return self.map

    def getGridPosition(self, coord):
        """
        :param coord: a quaternion
        :return: the grid's square (pair) corresponding to coord
        """
        row = floor((coord['X'] - self.xMin) / self.CELL_SIZE)
        col = floor((coord['Y'] - self.yMin) / self.CELL_SIZE)
        return row, col

    def getRealPosition(self, square):
        """
        :param square: a pair (grid's position)
        :return: the "real" position (quaternion) corresponding to square
        """
        x = (square[0] * self.CELL_SIZE + self.xMin) + self.CELL_SIZE / 2
        y = (square[1] * self.CELL_SIZE + self.yMin) + self.CELL_SIZE / 2
        return {'X': x, 'Y': y}

    def getNextLayer(self, layer, inside):
        """
        Given a layer (depth d) and all the squares already visited,
        computes the next layer (depth d+1) in the wave search
        :param layer: a list of squares (all at the same depth in the wave search)
        :param inside: a list of squares (that are part of a depth < d layer)
        :return: the list of squares corresponding to the next layer
        """
        nextLayer = []
        for square in layer:
            for (i, j) in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    neighbor = (square[0] + i, square[1] + j)
                    if not self.isOutOfBound(neighbor) and neighbor not in inside and not self.getState(neighbor) == self.OCCUPIED:
                        nextLayer.append(neighbor)
        return nextLayer

    def isOnBorder(self, square):
        """
        Decides whether a given square belongs to the border between known/unknown or not
        :param square: a pair
        :return: True iff square belongs to the border
        """
        if self.isOutOfBound(square) or self.getState(square) == self.UNKNOWN:
            return False
        for (i, j) in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                neighbor = (square[0] + i, square[1] + j)
                if not self.isOutOfBound(neighbor) and self.getState(neighbor) == self.UNKNOWN:
                    return True
        return False

    def findBorder(self, square):
        """
        Given a square, finds all the squares that belongs to the same 1 state border (either all empty or all occupied)
        :param square: a pair
        :return: the border (list of squares) associated to square, and squares belonging to the neighboring borders
        """
        border = []
        ends = []
        stack = [square]
        while len(stack) != 0:
            square = stack.pop()
            if square not in border:
                border.append(square)
                for i in range(-1, 2):
                    for j in range(-1, 2):
                        neighbor = (square[0] + i, square[1] + j)
                        if self.isOnBorder(neighbor) and neighbor not in border:
                            if self.getState(neighbor) == self.getState(square):
                                stack.append(neighbor)
                            else:
                                ends.append(neighbor)
        ends = list(set(ends))
        return border, ends
