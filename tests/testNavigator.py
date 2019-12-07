import sys
sys.path.append("../src/")
from Navigator import Navigator
from Cartographer import Cartographer
from robot import Robot
from Controller import Controller
import random

def testComputePath():
    robot = Robot()
    xMin = -10; xMax = 10; yMin = -10; yMax = 10
    cartographer = Cartographer(xMin, xMax, yMin, yMax, True)
    controller = Controller(cartographer)
    navigator = Navigator(controller, cartographer)
    dest = (random.randint(0, xMax - xMin), random.randint(0, yMax - yMin))
    print(dest)
    path = navigator.computePath(robot, dest)
    print(path)


if __name__ == "__main__":
    testComputePath()