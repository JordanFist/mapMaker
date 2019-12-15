import sys
sys.path.append("../src/")
from robot import Robot
from Cartographer import Cartographer
from show_map import ShowMap
from robot import Robot
from PlanningModule import PlanningModule
from Navigator import Navigator
from Controller import Controller
from matplotlib import pyplot as plt


def testPlanningModule():
    robot = Robot()
    cartographer = Cartographer(-20, 20, -20, 20, True)
    cartographer.update(robot)
    controller = Controller(cartographer)
    navigator = Navigator(controller, cartographer)
    planningModule = PlanningModule(cartographer, navigator, controller)

    plt.figure()
    image = cartographer.showMap.getImage()
    plt.imshow(image)

    dest = planningModule.pickDestination(robot)
    plt.plot(dest[1], dest[0], 'gs')

    pos = cartographer.getGridPosition(robot.getPosition())
    plt.plot(pos[1], pos[0], 'rs', markersize=6)
    plt.show()



if __name__ == "__main__":
    testPlanningModule()