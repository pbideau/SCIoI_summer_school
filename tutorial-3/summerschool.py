from buildhat import Motor
import time
from time import sleep
from threading import Timer, Thread
import numpy as np
from multiprocessing import Process, Queue
import queue
import psutil

from vis import Visualizer

import matplotlib as mpl
mpl.use('Agg')
import matplotlib.pyplot as plt

from planner import *
from bezier import *
from robot import Robot
from perception import Perception

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

        print(self.motor_left.description)
        print(self.motor_right.description)

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

        # print(move_left, move_right)

        if self.state_estimate_time_last is not None:
            dt = t - self.state_estimate_time_last
            # convert to rad/s
            ul = np.radians(move_left) / dt
            ur = np.radians(move_right) / dt
            self.propagate([ul, ur], dt)
            self.state_estimate_data.append([time.time() - self.starttime, ul, ur])

            print(self.state)

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

    # def forward(self):
    #     self.motor_left.start(50)
    #     self.motor_right.start(-50)

    # def back(self):
    #     self.motor_left.start(-50)
    #     self.motor_right.start(50)

    # def left(self):
    #     self.motor_left.start(50)
    #     self.motor_right.start(50)

    # def right(self):
    #     self.motor_left.start(-50)
    #     self.motor_right.start(-50)

# class PerceptionThread(Thread):
#     def __init__(self):
#         super().__init__()
#         self.rel_pos = []
#         self.perception = Perception()

#     def run(self):
#         while True:
#             self.rel_pos = Process(target=self.perception.sense_relative_positions)
#             print("NEW REL POS")


def perception_process(q):
    # https://stackoverflow.com/questions/23060383/lowering-process-priority-of-multiprocessing-pool-on-windows
    parent = psutil.Process()
    parent.nice(20)
    for child in parent.children():
        child.nice(20)

    perception = Perception()
    while True:
        rel_pos = perception.sense_relative_positions()
        q.put(rel_pos)
        print("NEW REL POS")



def main():

    rel_pos_queue = Queue()
    perception = Process(target=perception_process, args=(rel_pos_queue,))
    perception.start()
    # wait until camera is ready
    rel_pos_queue.get()

    # m = RobotModel()
    # state_i, _, _ = reference_circle(m, 0, 0.2)
    state_i = [0,0,0]
    robot = LegoRobot(state_i)


    # perception = PerceptionThread()
    # perception.start()

    # ts = np.arange(0,10,0.1)
    # states = np.empty((len(ts)+1, 3))
    # states_d = np.empty((len(ts)+1, 3))
    # r = Robot([0,0,0])
    # v.add_robot("r", r)
    # states[0] = r.state
    # for k, t in enumerate(ts):

    # motion planning

# # Define and solve an optimization problem

#     q_start = [0,0,0]
#     q_goal = [0.2,0.2,0]
#     v = 1

#     p = cp.Variable((4,2))

#     cost = cp.norm2(3*(p[0]-p[1]-p[2]+p[3]))
#     prob = cp.Problem(
#         cp.Minimize(cost),
#         [
#             bezier(p, 0) == q_start[0:2],
#             bezier_d(p, 0) == [v*np.cos(q_start[2]), v*np.sin(q_start[2])],
#             bezier(p, 1) == q_goal[0:2]
#         ]
#     )
#     prob.solve()
#     print(p.value)

#     # Plot the result
#     fig, ax = plt.subplots()
#     ax.set_aspect(1)
#     ax.scatter(p.value[:,0], p.value[:,1])
#     ts = np.linspace(0, 1, 100)
#     ps = np.empty((len(ts), 2))
#     for k, t in enumerate(ts):
#         ps[k] = bezier(p.value, t)
#     ax.plot(ps[:,0], ps[:,1])

#     plt.savefig("opt.pdf")


    ##

    goal = np.array([1.0,0,0])

    v = Visualizer()

    states = []
    states_d = []
    actions_d = []
    ts = []
    rel_pos = []

    state_d = [0, 0, 0]
    v_d = 0
    omega_d = 0

    while True:
        t = time.time() - robot.starttime
        if t > 60:
            break
        print(t)

        try:
            rel_pos = rel_pos_queue.get_nowait()
            v.add_image("camera", "perception.png")
        except queue.Empty:
            pass

        # compute transformation matrix
        x, y, theta = robot.state
        T = np.array([
            [np.cos(theta), -np.sin(theta), x],
            [np.sin(theta),  np.cos(theta), y],
            [0, 0, 1]])
        
        # the scipy voronoi code only works for at least three robots -> add dummys if needed
        if len(rel_pos) < 2:
            rel_pos.append([5, 5])
        if len(rel_pos) < 2:
            rel_pos.append([5, -5])

        # transform to world coordinates
        world_pos = [robot.state]
        for p in rel_pos:
            wp = T @ np.array([p[0], p[1], 1])
            world_pos.append(np.array([wp[0], wp[1], 0]))

        print("wp", world_pos)

        vstates = np.array(world_pos)

        vrobots = [Robot(s) for s in vstates]
        for k, r in enumerate(vrobots):
            v.add_robot("r{}".format(k), r)
        v.update_robots()

        p, vor = plan(vstates, 0, goal)
        if p is not None:
            v.add_bezier("plan", p, 0x0000ff00)

            line_segments = extract_line_segments(vor)
            v.add_line_segments2d("voronoi", line_segments)

            state_d, v_d, omega_d = reference_bezier(robot, p, 0.1*t)
        else:
            print("Warning: infeasible optimization")

        print("d", state_d)

        ts.append(t)
        states.append(robot.state)
        states_d.append(state_d)
        action = robot.controller(robot.state, state_d, v_d, omega_d, K_x=10, K_y=10, K_theta=30)
        action = np.clip(action, -10, 10)
        # action = [50*np.cos(1.0*t), 50*np.sin(1.0*t)]
        actions_d.append(action)
        robot.apply_action(action)
        # states[k+1] = r.state
        # v.update_robots()

        vrobots[0].state = robot.state
        v.update_robots()

        sleep(0.1)

    ts = np.array(ts)
    states = np.array(states)
    states_d = np.array(states_d)

    fig, axs = fig, ax = plt.subplots(3,1)
    for k in range(3):
        axs[k].plot(ts, states[:,k])
        axs[k].plot(ts, states_d[:,k], label="desired")

    plt.legend()

    plt.savefig("controller.pdf")

    actions_d = np.array(actions_d)
    fig, axs = fig, ax = plt.subplots(2,1)
    d = np.array(robot.motor_left_data)
    axs[0].plot(d[:,0], d[:,1]) # speed over time
    axs[0].plot(ts, actions_d[:,0], label="desired left") # speed over time

    d = np.array(robot.state_estimate_data)
    axs[0].plot(d[:,0], 4.5*d[:,1]) # speed over time

    d = np.array(robot.motor_right_data)
    axs[1].plot(d[:,0], d[:,1]) # speed over time
    axs[1].plot(ts, -actions_d[:,1], label="desired right") # speed over time

    d = np.array(robot.state_estimate_data)
    axs[1].plot(d[:,0], 4.5*-d[:,2]) # speed over time

    plt.legend()

    plt.savefig("actions.pdf")


    # robot.forward()
    # sleep(5)
    # robot.back()
    # sleep(1)
    # robot.right()
    # sleep(1)
    # robot.left()
    # sleep(1)
    robot.stop()

if __name__ == "__main__":
    main()
