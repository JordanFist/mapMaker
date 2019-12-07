class Navigator:
    def __init__(self, controller, cartographer):
        self.cartographer = cartographer
        self.controller = controller

    def computePath(self, robot, dest):
        # Returns a list of coordinate squares
        wave = self.wave(robot, dest)
        path = self.findPath(len(wave), wave, [dest])
        # If no path is found
        if path == None:
            return None
        path.reverse()
        return path

    def findPath(self, depth, wave, path):
        if depth == 0:
            return path
        goal = path[-1]
        neighbors = self.findNeighbors(goal, wave[depth - 1])
        if len(neighbors) == 0:
            path.pop()
            return None
        for neighbor in neighbors:
            path.append(neighbor)
            pathTmp = self.findPath(depth - 1, wave, path)
            if pathTmp:
                return pathTmp
        return None

    def findNeighbors(self, goal, layer):
        neighbors = set()
        for i in range(-1, 2):
            for j in range(-1, 2):
                if abs(i) + abs(j) != 0:
                    neighbors.add((goal[0] + i, goal[1] + j))
        neighborSet = neighbors.intersection(set(layer))
        return list(neighborSet)

    def wave(self, robot, dest):
        wave = []
        inside = [self.cartographer.getGridPosition(robot.getPosition())]
        nextBorder = self.cartographer.getNextBorder(inside, [])
        while dest not in nextBorder and nextBorder != []:
            wave.append(nextBorder)
            inside += nextBorder
            nextBorder = self.cartographer.getNextBorder(nextBorder, inside)
            for layer in wave:
                nextBorder = list(set(nextBorder) - set(layer))
        return wave

    def convertPath(self, path):
        newPath = []
        for square in path:
            newPath.append(self.cartographer.getRealPosition(square))
        return newPath

    def followThePath(self, robot, path):
        pass

