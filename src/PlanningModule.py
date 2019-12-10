class PlanningModule:

    def __init__(self, cartographer, navigator, controller):
        self.cartographer = cartographer
        self.navigator = navigator
    
    @staticmethod
    def getDistanceSquares(s, t):
        return sqrt((s[0] - t[0]) ** 2 + (s[1] - t[1]) ** 2)

    def pickDestination(self, robot):
        borders = self.getBorders(robot)
        robotPosition = self.cartographer.getGridPosition(robot.getPosition())
        centroids = []
        for border in borders:
            xSum = ySum = 0
            for square in border:
                xSum += square[0]
                ySum += square[1]
            centroids.append((xSum // len(maxLenBorder), ySum // len(maxLenBorder)))
        closestCentroid = centroids[0]
        for centroid in centroids:
            if getDistanceSquares(closestCentroid, robotPosition) < getDistanceSquares(centroid, robotPosition):
                closestCentroid = centroid
        return closestCentroid

    def move(self, robot):
        # Needs navigator
        pass

    def EndInBorders(self, end, borders):
        for border in borders:
            for square in border:
                if square == end:
                    return True
        return False

    def getBorders(self, robot):
        emptyBorders = []
        occupiedBorders = []
        borderSquare = self.cartographer.findBorderSquare(cartographer.getGridPosition(robot.getPosition()), set())
        border, ends = self.cartographer.findBorder(borderSquare)
        # Particular case in which all the border squares have the same state
        if len(ends) == 0:
            return [border]
        while not (self.EndInBorders(ends[0], emptyBorders + occupiedBorders) and self.EndInBorders(ends[1], emptyBorders + occupiedBorders)):
            if self.cartographer.getState(border[0]) == self.cartographer.EMPTY:
                emptyBorders.append(border)
            else:
                occupiedBorders.append(border)

            if not self.EndInBorders(end[0], emptyBorders + occupiedBorders):
                border, ends = self.cartographer.findBorder(ends[0])
            elif not self.EndInBorders(end[1], emptyBorders + occupiedBorders):
                border, ends = self.cartographer.findBorder(ends[1])
        return emptyBorders
