from Cartographer import Cartographer
from PlanningModule import PlanningModule
from Navigator import Navigator
from Controller import Controller
from robot import Robot
from time import time


def main():
    startTime = time()
    robot = Robot()
    cartographer = Cartographer(-20, 20, -20, 20, True)
    cartographer.update(robot)
    controller = Controller(cartographer)
    navigator = Navigator(controller, cartographer)
    planningModule = PlanningModule(cartographer, navigator, controller)
    while True:
        if not planningModule.move(robot):
            break
    cartographer.showMap.close()
    print("time: ", time() - startTime)

if __name__ == "__main__":
    main()