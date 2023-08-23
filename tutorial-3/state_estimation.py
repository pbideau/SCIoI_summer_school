from buildhat import Motor
import time
from time import sleep
from threading import Timer
import numpy as np

from vis import Visualizer
from robot import Robot

class RepeatTimer(Timer):
    def run(self):
        while not self.finished.wait(self.interval):
            self.function(*self.args, **self.kwargs)


class LegoRobot(Robot):
    def __init__(self, state=[0,0,0]):
        super().__init__(state=state, r=5.5/2/100, L=11/100)

        self.motor_left = Motor('C')
        # self.motor_left.set_speed_unit_rpm(True)
        self.pos_left = self.motor_left.get_aposition()
        self.pos_left_last = self.pos_left
        self.motor_left.when_rotated = self.handle_motor_left
        self.motor_right = Motor('D')
        # self.motor_left.set_speed_unit_rpm(True)
        self.motor_left.interval = 10 # ms
        self.pos_right = self.motor_right.get_aposition()
        self.pos_right_last = self.pos_right
        self.motor_right.when_rotated = self.handle_motor_right
        self.motor_right.interval = 10 # ms

        self.timer = RepeatTimer(0.05, self.state_estimate)
        self.timer.start()
        self.starttime = time.time()
        self.motor_left_data = []
        self.motor_right_data = []
        self.state_estimate_data = []
        self.state_estimate_time_last = None

    def state_estimate(self):
        t = time.time()

        move_left = self.pos_left - self.pos_left_last
        move_left = (move_left + 180) % 360 - 180 # in deg
        # one motor is inverted -> correct it here
        move_left = -move_left

        move_right = self.pos_right - self.pos_right_last
        move_right = (move_right + 180) % 360 - 180 # in deg

        if self.state_estimate_time_last is not None:
            dt = t - self.state_estimate_time_last
            # convert to rad/s
            ul = np.radians(move_left) / dt
            ur = np.radians(move_right) / dt
            self.state_estimate_data.append([time.time() - self.starttime, ul, ur])

            # TODO: use the robot dynamics to update the state estimate
            #       Note that LegoRobot inherits from Robot and can use all the methods in there


        self.pos_left_last = self.pos_left
        self.pos_right_last = self.pos_right
        self.state_estimate_time_last = t



    def handle_motor_left(self, speed, pos, apos):
        """Motor data

        :param speed: Speed of motor
        :param pos: Position of motor
        :param apos: Absolute position of motor
        """
        self.pos_left = apos
        self.motor_left_data.append([time.time() - self.starttime, speed, pos, apos])
        # print("Motor left", speed, pos, apos)

    def handle_motor_right(self, speed, pos, apos):
        """Motor data

        :param speed: Speed of motor
        :param pos: Position of motor
        :param apos: Absolute position of motor
        """
        self.pos_right = apos
        self.motor_right_data.append([time.time() - self.starttime, speed, pos, apos])
        # print("Motor right", speed, pos, apos)

    def apply_action(self, action):
        print(action)
        self.motor_left.start(-action[0])
        self.motor_right.start(action[1])

    def stop(self):
        self.motor_left.stop()
        self.motor_right.stop()


def main():

    state_i = [0,0,0]
    robot = LegoRobot(state_i)

    v = Visualizer()
    v.add_robot("r0", robot)

    while True:
        t = time.time() - robot.starttime
        if t > 30:
            break
        print(t)

        v.update_robots()
        robot.apply_action([5, 5])
        sleep(0.1)

    robot.stop()

if __name__ == "__main__":
    main()
