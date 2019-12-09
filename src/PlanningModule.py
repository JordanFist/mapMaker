class PlanningModule:

    def __init__(self, cartographer, navigator, controller):
        self.cartographer = cartographer
        self.navigator = navigator

    def pickDestination(self, robot):
        pass

    def move(self, robot):
        # Needs navigator
        pass

    def EndInBorders(self, end, borders):
        for border in borders:
            for square in border:
                if square == end:
                    return True
        return False

    def getBorders(self, robot): # check if extremity is none
        emptyBorders = occupiedBorders = extremities = []
        borderSquare = self.cartographer.findBorderSquare(cartographer.getGridPosition(robot.getPosition()), set())
        border, ends = self.cartographer.findBorder(borderSquare)
        borders.append(border)
        while not (self.EndInBorders(ends[0]) and self.EndInBorders(ends[1])):
            if self.cartographer.getState(border[0]) == self.cartographer.EMPTY:
                EmptyBorders.append(border)
                if not self.EndInBorders(end[0], occupiedBorders):
                    border, ends = self.cartographer.findBorder(ends[0])
                elif not self.EndInBorders(end[1], occupiedBorders):
                    border, ends = self.cartographer.findBorder(ends[1])
                    
            
            else:
                occupiedBorders.append(border)
                if not self.EndInBorders(end[0], emptyBorders):
                    border, ends = self.cartographer.findBorder(ends[0])
                elif not self.EndInBorders(end[1], emptyBorders):
                    border, ends = self.cartographer.findBorder(ends[1])