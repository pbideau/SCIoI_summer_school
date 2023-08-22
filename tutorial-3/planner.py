import numpy as np
import cvxpy as cp

from scipy.spatial import Voronoi
from voronoi_helper import *
from bezier import *

def reference_bezier(robot, p, t):
    t = np.clip(t, 0, 1)
    x, y = bezier(p, t)
    x_dot, y_dot = bezier_d(p, t)
    x_ddot, y_ddot = bezier_dd(p, t)
    return robot.diff_flatness(x, y, x_dot, y_dot, x_ddot, y_ddot)

def reference_circle(robot, t, r=5.0):
    x = r*np.cos(t)
    y = r*np.sin(t)
    x_dot = -r*np.sin(t)
    x_ddot = -r*np.cos(t)
    y_dot = r*np.cos(t)
    y_ddot = -r*np.sin(t)
    return robot.diff_flatness(x, y, x_dot, y_dot, x_ddot, y_ddot)


def plan(states, i, goal):
    vor = Voronoi(states[:,0:2])
    hyperspaces = extract_hyperspaces_per_point(vor)

    v = 0.1
    radius = 0.2

    p = cp.Variable((4,2))

    cost = cp.norm2(3*(p[0]-p[1]-p[2]+p[3])) + 0.1*cp.norm2(bezier(p, 1) - goal[0:2])
    constraints = [
        bezier(p, 0) == states[i,0:2],
        bezier_d(p, 0) == [v*np.cos(states[i,2]), v*np.sin(states[i,2])],
        # bezier(p, 1) == q_goal[0:2]
    ]
    for n, d in hyperspaces[i]:
        constraints.extend([
            p[0] @ n >= d+radius,
            p[1] @ n >= d+radius,
            p[2] @ n >= d+radius,
            p[3] @ n >= d+radius,
        ])

    prob = cp.Problem(
        cp.Minimize(cost),
        constraints
    )
    prob.solve()
    return p.value, vor