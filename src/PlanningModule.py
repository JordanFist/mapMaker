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
        self.MIN_BORDER_SIZE = 3
    
    @staticmethod
    def getDistanceSquares(s, t):
        return sqrt((s[0] - t[0]) ** 2 + (s[1] - t[1]) ** 2)

    @staticmethod
    def getMedian(border):
        rows = [square[0] for square in border]
        cols = [square[1] for square in border]
        rows.sort()
        cols.sort()
        middle = int(len(border) / 2)
        return (rows[middle], cols[middle])

    def getNonObstacle(self, squares):
        for square in squares:
            if self.cartographer.getState(square) != self.cartographer.OCCUPIED:
                return square
        return None

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
        medians = []
        for border in borders:
            if len(border) > self.MIN_BORDER_SIZE:
                medians.append(self.getMedian(border))
        # Pick the closest median to the robot that is not an obstacle
        closestMedian = self.getNonObstacle(medians)
        if not closestMedian:
            return None
        for median in medians:
            dist = self.getDistanceSquares(median, robotPosition)
            if dist < self.getDistanceSquares(closestMedian, robotPosition) \
                    and self.cartographer.getState(median) != self.cartographer.OCCUPIED:
                    closestMedian = median
        return closestMedian

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
        alreadyVisited = []

        for row in range(self.cartographer.getHeight()):
            for col in range(self.cartographer.getWidth()):
                if self.cartographer.isOnBorder((row, col)) and not (row, col) in alreadyVisited \
                        and self.cartographer.getState((row, col)) != self.cartographer.OCCUPIED:
                    toVisit.append((row, col))

                while len(toVisit) != 0:
                    square = toVisit.pop()
                    if not self.EndInBorders(square, emptyBorders + occupiedBorders):
                        border, ends = self.cartographer.findBorder(square)
                        toVisit += ends
                        if self.cartographer.getState(square) == self.cartographer.EMPTY:
                            emptyBorders.append(border)
                            alreadyVisited += border
                        else:
                            occupiedBorders.append(border)
        return emptyBorders

    def move(self, robot):
        destination = self.pickDestination(robot)
        if not destination:
            return False
        self.navigator.followThePath(robot, destination)
        return True
