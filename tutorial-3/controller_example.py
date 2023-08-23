import time
from time import sleep
import numpy as np

from vis import Visualizer

import matplotlib as mpl
mpl.use('Agg')
import matplotlib.pyplot as plt

from planner import *
from bezier import *
from state_estimation import LegoRobot

def main():

    state_i = [0,0,0]
    robot = LegoRobot(state_i)

    goal = np.array([1.0,0,0])

    v = Visualizer()
    v.add_robot("r0", robot)

    states = []
    states_d = []
    actions_d = []
    ts = []

    state_d = [0, 0, 0]
    v_d = 0
    omega_d = 0

    while True:
        t = time.time() - robot.starttime
        if t > 60:
            break
        v.update_robots()

        p = plan_bezier(robot.state, goal)

        if p is not None:
            v.add_bezier("plan", p, 0x0000ff00)
            state_d, v_d, omega_d = reference_bezier(robot, p, 0.1*t)
        else:
            print("Warning: infeasible optimization")

        print("d", state_d)

        ts.append(t)
        states.append(robot.state)
        states_d.append(state_d)
        action = robot.controller(robot.state, state_d, v_d, omega_d, K_x=10, K_y=10, K_theta=30)
        action = np.clip(action, -10, 10)
        actions_d.append(action)
        robot.apply_action(action)

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
