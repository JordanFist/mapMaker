import sys
sys.path.append("../src/")
from src.Cartographer import Cartographer
from src.show_map import ShowMap
from src.robot import Robot
from time import sleep

def testUpdate():
    robot = Robot()
    cartographer = Cartographer(-40, 40, -40, 40, False)
    cartographer.update(robot)
    cartographer.showMap.close()

if __name__ == "__main__":
    testUpdate()