from math import sqrt

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
        robotPosition = self.cartographer.getGridPosition(robot.getPosition())
        # Compute the centroid of each border
        centroids = []
        for border in borders:
            xSum = ySum = 0
            for square in border:
                xSum += square[0]
                ySum += square[1]
            centroids.append((xSum // len(border), ySum // len(border)))
        # Pick the closest centroid to the robot
        closestCentroid = centroids[0]
        for centroid in centroids:
            if self.getDistanceSquares(closestCentroid, robotPosition) < self.getDistanceSquares(centroid, robotPosition):
                closestCentroid = centroid
        return closestCentroid

    def EndInBorders(self, end, borders):
        """
        If one of the ends' square belongs to a non visited border, returns it
        Otherwise returns None
        :param end: a square
        :param borders: a list of borders
        :return: a square if not in borders otherwise None
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
        return emptyBorders

    def move(self, robot):
        destination = self.pickDestination(robot)
        print(destination)
        if destination == self.oldDestination:
            return False
        self.navigator.followThePath(robot, destination)
        self.oldDestination = destination
        return True
