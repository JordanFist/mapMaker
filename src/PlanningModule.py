from math import sqrt
from random import choice


class PlanningModule:
    """
    This class represents the mission planner : it picks a destination using the frontier based exploration algorithm
    and then gives this destination to the navigator
    """

    def __init__(self, cartographer, navigator, controller):
        self.cartographer = cartographer
        self.navigator = navigator
        self.controller = controller
        self.oldDestination = None
        self.attempt = 0
    
    @staticmethod
    def getDistanceSquares(s, t):
        return sqrt((s[0] - t[0]) ** 2 + (s[1] - t[1]) ** 2)

    def pickDestination(self, robot):
        """
        Choose a new destination, using the frontier based exploration algorithm
        Here a border is composed of empty squares having at least 1 unknown neighbor (Von Neumann)
        :param robot: Robot object
        :return: the new destination
        """
        borders = self.getBorders(robot)
        if not borders:
            return None
        robotPosition = self.cartographer.getGridPosition(robot.getPosition())
        # Compute the centroid of each border
        centroids = []
        for border in borders:
            xSum = ySum = 0
            for square in border:
                xSum += square[0]
                ySum += square[1]
            centroids.append((xSum // len(border), ySum // len(border)))

        #self.cartographer.showMap.yellow_points = centroids

        # Pick the closest centroid to the robot
        closestCentroid = centroids[0]
        for centroid in centroids:
            if self.getDistanceSquares(centroid, robotPosition) < self.getDistanceSquares(closestCentroid, robotPosition):
                if self.cartographer.getState(centroid) != self.cartographer.OCCUPIED:
                    closestCentroid = centroid
        # if self.cartographer.getState(closestCentroid) == self.cartographer.OCCUPIED:
        #     return self.oldDestination
        return closestCentroid

    def EndInBorders(self, end, borders):
        """
        :param end: a square
        :param borders: a list of borders
        :return: True iff end is in one of the borders
        """
        for border in borders:
            if end in border:
                return True
        return False

    def getBorders(self, robot):
        """
        Retrieves all the borders
        Here a border may be composed of either empty squares or occupied squares
        :param robot: Robot object
        :return: the list of all the borders composed of empty squares
        """
        emptyBorders = []
        occupiedBorders = []
        toVisit = []

        borderSquare = self.cartographer.findBorderSquare(self.cartographer.getGridPosition(robot.getPosition()))
        if not borderSquare:
            print("pas de border square")
            return []
        border, ends = self.cartographer.findBorder(borderSquare)
        if self.cartographer.getState(border[0]) == self.cartographer.EMPTY:
            emptyBorders.append(border)
        else:
            occupiedBorders.append(border)
        toVisit += ends
        # Particular case in which all the border squares have the same state (empty or occupied)
        if len(toVisit) == 0:
            return [border]

        while len(toVisit) != 0:
            end = toVisit.pop()
            if not self.EndInBorders(end, emptyBorders + occupiedBorders):
                border, ends = self.cartographer.findBorder(end)
                toVisit += ends
                if self.cartographer.getState(border[0]) == self.cartographer.EMPTY:
                    emptyBorders.append(border)
                else:
                    occupiedBorders.append(border)
        self.cartographer.showMap.blue_points = []
        self.cartographer.showMap.green_points = []
        for border in emptyBorders:
            self.cartographer.showMap.green_points += border
        for border in occupiedBorders:
            self.cartographer.showMap.blue_points += border
        return emptyBorders

    def move(self, robot):
        destination = self.pickDestination(robot)
        if not destination:
            return False
        self.cartographer.showMap.yellow_points = [destination]
        # if destination == self.oldDestination:
        if destination == self.oldDestination:
            print("same destination")
            self.attempt += 1
            if self.attempt == 3:
                self.cartographer.map[destination[0]][destination[1]] = 15
                self.attempt = 0
        else:
            self.attempt = 0
        if self.navigator.reachedDestination(robot, destination):
            self.controller.wander(robot)
        else:
            self.navigator.followThePath(robot, destination)
            self.oldDestination = destination
        return True
