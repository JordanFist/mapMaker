class PlanningModule:
    """
    This class represents the mission planner : it picks a destination using the frontier based exploration algorithm
    and then gives this destination to the navigator
    """

    def __init__(self, cartographer, navigator, controller):
        self.cartographer = cartographer
        self.navigator = navigator
    
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
            centroids.append((xSum // len(maxLenBorder), ySum // len(maxLenBorder)))
        # Pick the closest centroid to the robot
        closestCentroid = centroids[0]
        for centroid in centroids:
            if getDistanceSquares(closestCentroid, robotPosition) < getDistanceSquares(centroid, robotPosition):
                closestCentroid = centroid
        return closestCentroid

    def EndInBorders(self, end, borders):
        """
        Decides whether end belongs to one of the borders given
        :param end: a square
        :param borders: a list of borders
        :return: True iff end in one of the borders
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
        borderSquare = self.cartographer.findBorderSquare(cartographer.getGridPosition(robot.getPosition()), set())
        border, ends = self.cartographer.findBorder(borderSquare)
        # Particular case in which all the border squares have the same state (empty or occupied)
        if len(ends) == 0:
            return [border]
        # Assuming a border has 2 neighboring borders, stop when the 2 neighbors have already been visited
        while not (self.EndInBorders(ends[0], emptyBorders + occupiedBorders) and self.EndInBorders(ends[1], emptyBorders + occupiedBorders)):
            if self.cartographer.getState(border[0]) == self.cartographer.EMPTY:
                emptyBorders.append(border)
            else:
                occupiedBorders.append(border)

            # Check one of the neighboring border (if not already visited)
            if not self.EndInBorders(end[0], emptyBorders + occupiedBorders):
                border, ends = self.cartographer.findBorder(ends[0])
            elif not self.EndInBorders(end[1], emptyBorders + occupiedBorders):
                border, ends = self.cartographer.findBorder(ends[1])
        return emptyBorders

    def move(self, robot):
        # Needs navigator
        pass
