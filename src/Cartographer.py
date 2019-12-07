from show_map import ShowMap
import numpy as np
from math import floor


class Cartographer:

    def __init__(self, xMin, xMax, yMin, yMax, showGUI):
        self.CELL_SIZE = 1
        self.EMPTY = 0
        self.OCCUPIED = 1
        self.UNKNOWN = -1

        self.xMin = xMin
        self.xMax = xMax
        self.yMin = yMin
        self.yMax = yMax
        self.showMap = ShowMap(self.getHeight(), self.getWidth(), showGUI)
        self.map = np.ones((self.getHeight(), self.getWidth())) * self.UNKNOWN

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

    def getGridPosition(self, coord):
        col = floor((coord['X'] - self.xMin) / self.CELL_SIZE)
        row = floor((coord['Y'] - self.yMin) / self.CELL_SIZE)
        return (row, col)

    def getRealPosition(self, square):
        x = (square[0] * self.CELL_SIZE + self.xMin) + self.CELL_SIZE / 2
        y = (square[1] * self.CELL_SIZE + self.yMin) + self.CELL_SIZE / 2
        return {'X': x, 'Y': y}

    def getNextBorder(self, border, inside):
        nextBorder = []
        for cell in border:
            for (i, j) in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    neighborRow, neighborCol = cell[0] + i, cell[1] + j
                    if (not self.isOutOfBound(neighborRow, neighborCol)
                            and (neighborRow, neighborCol) not in inside and not self.isObstacle(neighborRow, neighborCol)):
                        nextBorder.append((neighborRow, neighborCol))
        return nextBorder
