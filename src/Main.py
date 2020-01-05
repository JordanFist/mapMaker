from Cartographer import Cartographer
from PlanningModule import PlanningModule
from Navigator import Navigator
from Controller import Controller
from robot import Robot
from sys import argv


def main():
    _, url, x1, y1, x2, y2, showGUI = argv
    robot = Robot(url)
    cartographer = Cartographer(int(x1), int(x2), int(y1), int(y2), int(showGUI))
    cartographer.update(robot)
    controller = Controller(cartographer)
    navigator = Navigator(controller, cartographer)
    planningModule = PlanningModule(cartographer, navigator, controller)
    while True:
        if not planningModule.move(robot):
            break
    cartographer.showMap.close()

if __name__ == "__main__":
    main()