import sys
sys.path.append("../src/")
from Cartographer import Cartographer
from show_map import ShowMap
from robot import Robot
from time import sleep

def testUpdate():
    robot = Robot()
    cartographer = Cartographer(-40, 40, -40, 40, False)
    cartographer.update(robot)
    cartographer.showMap.close()

def passed():
    print(__file__, '\x1b[6;30;42m' + 'Success!' + '\x1b[0m')

if __name__ == "__main__":
    testUpdate()