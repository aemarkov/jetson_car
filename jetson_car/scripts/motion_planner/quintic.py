import numpy as np

def calc_coefs(s1, s2, T):
    A = np.array([[   0,       0,       0,      0,    0, 1],
              [   0,       0,       0,      0,    1, 0],
              [   0,       0,       0,      1,    0, 0],
              [   T**5,    T**4,    T**3,   T**2, T, 1],
              [ 5*T**4,  4*T**3,  3*T**2, 2*T,    1, 0], 
              [20*T**3, 12*T**2,  6*T,      1,    0, 0]])
    B = np.array([s1[0], s1[1], s1[2], s2[0], s2[1], s2[2]])
    return np.linalg.solve(A, B)    

def calc_quintic(coefs, x):
    return coefs[0]*x**5 + coefs[1]*x**4 + coefs[2]*x**3 + coefs[3]*x**2 + coefs[4]*x + coefs[5]

def calc_dquintic(coefs, x):
    return 5*coefs[0]*x**4 + 4*coefs[1]*x**3 + 3*coefs[2]*x**2 + 2*coefs[3]*x + coefs[4]

def calc_ddquintic(coefs, x):
    return 20*coefs[0]*x**3 + 12*coefs[1]*x**2 + 6*coefs[2]*x + coefs[3]

def interpolate(coefs, time):
    n = len(time)
    x = np.zeros(n)
    dx = np.zeros(n)
    ddx = np.zeros(n)

    for i, t in enumerate(time):
        x[i] = calc_quintic(coefs, t)
        dx[i] = calc_dquintic(coefs, t)
        ddx[i] = calc_ddquintic(coefs, t)

    return x, dx, ddx