import numpy as np
import warnings
warnings.filterwarnings("error")
from scipy.optimize import fsolve

# def func(x):
#     return (0/6.6)**4 + (0/6.6)**4 + (-3*x/6.6)**4 - 1
def callSolve(Ox,Oy,Oz,Px,Py,Pz):
    def func(x):
        return ((Ox + (Px-Ox)*x)/6.6)**4 + ((Oy + (Py-Oy)*x)/6.6)**4 + ((Oz + (Pz-Oz)*x)/6.6)**4 - 1
    try:
        root = fsolve(func, 0.5)
        print(root)
    except RuntimeWarning:
        print("Runtime")
        root = fsolve(func, 1)
        print(root)

callSolve(0,0,0,0,0,3)
# def func(x):
#     return ((Ox + (Px-Ox)*x)/6.6)**4 + ((Oy + (Py-Oy)*x)/6.6)**4 + ((Oz + (Pz-Oz)*x)/6.6)**4 - 1
# try:
#     root = fsolve(func, 0.5)
#     print(root)
# except RuntimeWarning:
#     print("Runtime")
#     root = fsolve(func, 1)
#     print(root)

# from scipy.optimize import fsolve

# def equation(t):
#     return 192*t**4 - 256*t + 384*t**2 - 256*t**3 - (6.6**4 - 64)

# # Initial guess for t
# initial_guess = 0.62

# # Find the root of the equation
# t_solution = fsolve(equation, initial_guess)

# print("Approximate solution for t:", t_solution)