# This file contains functions unchanged from tutorial-1

import numpy as np

class Robot:
    def __init__(self, state=[0,0,0], r=5.5/2/100, L=11/100):
        self.state = state
        self.r = r
        self.L = L

    # dynamics for a given state and action
    def f(self, state, action):
        return np.array([
            self.r/2*(action[0] + action[1]) * np.cos(state[2]),
            self.r/2*(action[0] + action[1]) * np.sin(state[2]),
            self.r/self.L*(action[1] - action[0])
        ])

    def propagate(self, action, dt=0.1):
        self.state = self.state + self.f(self.state, action) *dt

    # input: v, omega; output: u_l, u_r
    def action_v_omega_to_ul_ur(self, action):
        v, omega = action
        u_l = (2*v - self.L*omega)/(2*self.r)
        u_r = (2*v + self.L*omega)/(2*self.r)
        return np.array([u_l, u_r])

    def controller(self, state, state_desired, v_d, omega_d, K_x=1, K_y=1, K_theta=1):
        # compute errors
        x, y, theta = state
        x_d, y_d, theta_d = state_desired
        x_e = (x_d-x)*np.cos(theta)+(y_d-y)*np.sin(theta)
        y_e = -(x_d-x)*np.sin(theta)+(y_d-y)*np.cos(theta)
        theta_e = theta_d - theta
        # compute control output (v,w space)
        v_ctrl = v_d * np.cos(theta_e) + K_x * x_e
        omega_ctrl = omega_d + v_d * (K_y * y_e + K_theta*np.sin(theta_e))
        # convert control output to (u_l, u_r space)
        return self.action_v_omega_to_ul_ur([v_ctrl, omega_ctrl])

    # input: x, y and derivatives; output: [x,y,theta], v, omega
    def diff_flatness(self, x, y, x_dot, y_dot, x_ddot, y_ddot):
        theta = np.arctan2(y_dot, x_dot)
        v = np.sqrt(y_dot**2 + x_dot**2)
        omega = (x_dot*y_ddot - y_dot*x_ddot) / (x_dot**2 + y_dot**2)
        return [x, y, theta], v, omega