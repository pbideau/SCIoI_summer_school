
# Define Bezier curve functions
def bezier(p, t):
    return (1-t)**3*p[0] + 3*t*(1-t)**2*p[1]+3*t**2*(1-t)*p[2]+t**3*p[3]

def bezier_d(p, t):
    return 3*(1-t)**2 *(p[1] - p[0]) + 6*(1-t)*t*(p[2] - p[1]) + 3*t**2*(p[3] - p[2])

def bezier_dd(p, t):
    return 6*(1-t)*(p[2]-2*p[1]+p[0])+6*t*(p[3]-2*p[2]+p[1])