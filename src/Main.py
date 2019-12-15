from robot import Robot
from Cartographer import Cartographer
from PlanningModule import PlanningModule
from Navigator import Navigator
from Controller import Controller
from show_map import ShowMap
from robot import Robot

def main():
    robot = Robot()
    cartographer = Cartographer(-20, 20, -20, 20, True)
    cartographer.update(robot)
    # plt.figure()
    # image = cartographer.showMap.getImage()
    # plt.imshow(image)
    controller = Controller(cartographer)
    navigator = Navigator(controller, cartographer)
    planningModule = PlanningModule(cartographer, navigator, controller)
    while True:
        if not planningModule.move(robot):
            break
    cartographer.showMap.close()

if __name__ == "__main__":
    main()