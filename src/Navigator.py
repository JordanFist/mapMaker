from Computations import getDistance

class Navigator:
    """
    This class represents the navigator : its goal is to find a path from the robot to the given destination
    and feed the controller accordingly
    """
    def __init__(self, controller, cartographer):
        self.cartographer = cartographer
        self.controller = controller
        self.TOLERANCE = 2
        self.SEGMENT_LENGTH = 40

    def computePath(self, robot, dest):
        """
        Find a path from the robot to the given destination using the front wave algorithm
        :param robot: Robot object
        :param dest: a square given by the mission planner
        :return: a list of neighboring squares (Von Neumann), robot position not included
        """
        wave = self.wave(robot, dest)
        path = self.findPath(len(wave), wave, [dest])
        # If no path is found
        if not path:
            return None
        path.reverse()
        return path

    def findPath(self, depth, wave, path):
        """
        Given a depth "depth" in the wave search, find an relevant neighbor at "deth - 1" to complete the path
        :param depth: a depth in the wave search (distance in squares from the robot)
        :param wave: a list of layers (each layer is a list of empty neighboring squares all at the same depth)
        :param path: the current path from the destination to the given depth
        :return: a list of neighboring squares
        """
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
        """
        Finds the given square's neighbors which belong to the given layer
        :param goal: a square
        :param layer: a list of empty neighboring squares (all at the same depth in the wave search)
        :return: the list of goal's neighbors which belong to layer
        """
        neighbors = set()
        for i in range(-1, 2):
            for j in range(-1, 2):
                if abs(i) + abs(j) != 0:
                    neighbors.add((goal[0] + i, goal[1] + j))
        neighborSet = neighbors.intersection(set(layer))
        return list(neighborSet)

    def wave(self, robot, dest):
        """
        Find all the layers (lists of empty neighboring squares) from the robot to the destination
        :param robot: Robot object
        :param dest: square to reach
        :return: a list of the layers
        """
        wave = []
        inside = [self.cartographer.getGridPosition(robot.getPosition())]
        nextLayer = self.cartographer.getNextLayer(inside, [])
        while dest not in nextLayer and nextLayer != []:
            wave.append(nextLayer)
            inside += nextLayer
            nextLayer = self.cartographer.getNextLayer(nextLayer, inside)
            # remove redundant elements
            nextLayer = list(set(nextLayer))
        return wave

    def convertPath(self, path):
        """
        Convert a grid square path to real positions path
        :param path: list of squares
        :return: list of quaternions
        """
        newPath = []
        for square in path:
            newPath.append(self.cartographer.getRealPosition(square))
        return newPath

    def reachedDestination(self, robot, dest):
        """
        Decides whether the robot is close enough to its destination (according to the tolerance attribute)
        :param robot: Robot object
        :param dest: square to reach
        :return: True iff the robot is close enough
        """
        return getDistance(robot.getPosition(), self.cartographer.getRealPosition(dest)) < self.TOLERANCE

    def followThePath(self, robot, dest):
        """
        Compute a path between the robot and the destination and make sure the robot reach it
        :param robot: Robot object
        :param dest: square to reach
        """
        # If the robot has already reached the destination, a new destination needs to be computed
        if self.reachedDestination(robot, dest):
            print("reached")
            self.controller.wander(robot)
            return

        path = self.computePath(robot, dest)
        if not path:
            print("no path")
            self.controller.wander(robot)
            return
        self.cartographer.showMap.blue_points = path
        path = self.convertPath(path)

        offset = 0
        while not self.reachedDestination(robot, dest) \
                and self.cartographer.getState(dest) != self.cartographer.OCCUPIED \
                and offset < len(path):
            # Divide the path in segments
            if offset + self.SEGMENT_LENGTH <= len(path):
                path = path[offset:offset + self.SEGMENT_LENGTH]
            else:
                path = path[offset:]
            offset += self.SEGMENT_LENGTH
            if not self.controller.move(robot, path):
                self.controller.wander(robot)
                self.followThePath(robot, dest)
