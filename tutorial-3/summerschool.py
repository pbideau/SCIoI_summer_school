import time
from time import sleep
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
from state_estimation import LegoRobot
from perception import Perception


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
        
        # You may add additional "virtual robots" into the rel_pos array here
        # rel_pos.append([x,y,z])
        
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

        vstates = np.array(world_pos)

        vrobots = [Robot(s) for s in vstates]
        for k, r in enumerate(vrobots):
            v.add_robot("r{}".format(k), r)

        p, vor = plan(vstates, 0, goal)
        if p is not None:
            v.add_bezier("plan", p, 0x0000ff00)

            line_segments = extract_line_segments(vor)
            v.add_line_segments2d("voronoi", line_segments)

            state_d, v_d, omega_d = reference_bezier(robot, p, 0.1*t)
        else:
            print("Warning: infeasible optimization")

        ts.append(t)
        states.append(robot.state)
        states_d.append(state_d)
        action = robot.controller(robot.state, state_d, v_d, omega_d, K_x=10, K_y=10, K_theta=30)
        action = np.clip(action, -10, 10)
        actions_d.append(action)
        robot.apply_action(action)

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

    robot.stop()

if __name__ == "__main__":
    main()
