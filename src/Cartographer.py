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

    def isOutOfBound(self, square):
        return not (0 <= square[0] < self.getHeight() and 0 <= square[1] < self.getWidth())

    def getState(self, square):
        if self.map[square[0]][square[1]] == 0.5:
            return self.UNKNOWN
        elif self.map[square[0]][square[1]] > 0.5:
            return self.OCCUPIED
        else:
            return self.EMPTY

    def update(self):
        pass

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

    def getNextBorder(self, border, inside):
        nextBorder = []
        for cell in border:
            for (i, j) in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    neighbor = (cell[0] + i, cell[1] + j)
                    if not self.isOutOfBound(neighbor) and neighbor not in inside and not self.getState(neighbor) == self.OCCUPIED:
                        nextBorder.append((neighborRow, neighborCol))
        return nextBorder

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
        border = ends = []
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







