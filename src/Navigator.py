class Navigator:
    def __init__(self, controller, cartographer):
        self.cartographer = cartographer
        self.controller = controller

    def computePath(self, robot, dest):
        # Returns a list of coordinate squares
        path = [dest]
        wave = self.wave(robot, dest)
        wave.pop()
        while dest != self.cartographer.getPosition(robot):
            dest = self.findNeighbor(dest, wave.pop())
            path.append(dest)
        path.reverse()

    def findNeighbor(self, goal, layer):
        neighbors = set()
        for i in range(-1, 2):
            for j in range(-1, 2):
                if abs(i) + abs(j) != 0:
                    neighbors.add((goal[0] + i, goal[1] + j))
        neighborSet = neighbors.intersection(set(layer))
        return neighborSet.pop()

    def wave(self, robot, dest):
        wave = []
        inside = [self.cartographer.getPosition(robot)]
        nextBorder = self.cartographer.getNextBorder(inside, [])
        while dest not in nextBorder and nextBorder != []:
            wave.append(nextBorder)
            inside += nextBorder
            nextBorder = self.cartographer.getNextBorder(nextBorder, inside)
            for layer in wave:
                nextBorder = list(set(nextBorder) - set(layer))
        return wave

    def followThePath(self, robot, path):
        pass