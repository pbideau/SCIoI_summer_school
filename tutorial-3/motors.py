from buildhat import Motor
from time import sleep

class LegoRobot():
    def __init__(self):
        self.motor_left = Motor('C')
        self.motor_right = Motor('D')

    def stop(self):
        self.motor_left.stop()
        self.motor_right.stop()

    def forward(self):
        self.motor_left.start(-5)
        self.motor_right.start(5)

    def back(self):
        self.motor_left.start(5)
        self.motor_right.start(-5)

    def left(self):
        self.motor_left.start(5)
        self.motor_right.start(5)

    def right(self):
        self.motor_left.start(-5)
        self.motor_right.start(-5)

def main():

    robot = LegoRobot()

    robot.forward()
    sleep(2)
    # robot.back()
    # sleep(2)
    # robot.right()
    # sleep(2)
    # robot.left()
    # sleep(2)
    robot.stop()

if __name__ == "__main__":
    main()
