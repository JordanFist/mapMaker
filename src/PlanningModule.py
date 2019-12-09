class PlanningModule:

    def __init__(self, cartographer, navigator, controller):
        self.cartographer = cartographer
        self.navigator = navigator

    def pickDestination(self, robot):
        pass

    def move(self, robot):
        # Needs navigator
        pass

    def ExtremityInBorders(self, extremity, borders):
        for border in borders:
            for square in border:
                if square == extremity:
                    return True
        return False

    def getBorders(self, robot):
        borders = []
        borderSquare = self.cartographer.findBorderSquare(cartographer.getGridPosition(robot.getPosition()), set())
        border, extremities = self.cartographer.findBorder(borderSquare)
        while self.ExtremityInBorders(extremities):    #stop when 2 extremities in borders
            pass