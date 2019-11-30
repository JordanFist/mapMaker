from show_map import ShowMap
import numpy as np


class Cartographer:

    def __init__(self, xMin, xMax, yMin, yMax, showGUI):
        # CELL_SIZE must be between 0 and 1
        self.CELL_SIZE = 1
        self.EMPTY = 0
        self.OCCUPIED = 1
        self.UNKNOWN = -1

        self.xMin = xMin
        self.xMax = xMax
        self.yMin = yMin
        self.yMax = yMax
        self.showMap = ShowMap(self.getHeight(), self.getWidth(), showGUI)
        self.map = np.array((self.getHeight(), self.getWidth()))

    def getHeight(self):
        """
        :return: the number of squares in terms of height
        """
        return (self.yMax - self.yMin) / self.CELL_SIZE

    def getWidth(self):
        """
        :return: the number of squares in terms of width
        """
        return (self.xMax - self.xMin) / self.CELL_SIZE

    def isOutOfBound(self, row, col):
        return not (0 <= row < self.getHeight() and 0 <= col < self.getWidth())

    def isObstacle(self, row, col):
        return self.getState(row, col) == self.OCCUPIED

    def update(self):
        pass

    def getMap(self):
        return self.map

    def getState(self, row, col):
        return self.map[row][col]

    def getPosition(self, robot):
        col = (robot.getPosition()['X'] - self.xMin) / self.CELL_SIZE
        row = (self.yMax - robot.getPosition()['Y']) / self.CELL_SIZE
        return (row, col)

    def getNextBorder(self, border, inside):
        nextBorder = []
        for cell in border:
            row, col = cell[0], cell[1]
            for i in range(-1, 2):
                for j in range(-1, 2):
                    if (abs(i) + abs(j) != 0 and not self.isOutOfBound(row + i, col + j)
                            and cell not in inside and not self.isObstacle(row + i, col + j)):
                        nextBorder.append(cell)
        return nextBorder
