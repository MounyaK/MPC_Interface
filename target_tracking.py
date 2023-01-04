
from struct import Struct
from casadi import *
from numpy import zeros, array, eye, size, block, shape, dot, full
# from cplex import *

# from matplotlib import pyplot as plt

# System Declaration
# A = SX([ [1, 0, 0.5, 0], [0, 1, 0, 0.5], [0, 0, 1, 0], [0, 0, 0, 1] ])
# print("A= ", A)
# B = SX([ [0, 0], [0, 0], [0.5, 0], [0, 0.5] ])
# print("B= ", B)
# C = SX([ [1, 0, 0, 0], [0, 1, 0, 0] ])
# print("C= ", C)
# D = SX.zeros(2,2)
# print("D= ", D)
A = array([ [1, 0, 0.5, 0], [0, 1, 0, 0.5], [0, 0, 1, 0], [0, 0, 0, 1] ])
print("A= ", A)
B = array([ [0, 0], [0, 0], [0.5, 0], [0, 0.5] ])
print("B= ", B)
C = array([ [1, 0, 0, 0], [0, 1, 0, 0] ])
print("C= ", C)
D = zeros((2,2))
print("D= ", D)

print(size(B))
dx, du = shape(B)
dy = size(C, 0)
print("dx= ", dx)
print("du= ", du)
print("dy= ", dy)

# Constraints
umin = -10
umax = 10
delta_u_min = -0.1
delta_u_max = 0.1
ymin = -10;
ymax = 10;

#Tuning matrixes
Q = eye(4)
print("Q=", Q)
# P = block([Q/10, 10*Q])
# P = [Q*10, Q/10]
Qy = eye(dy)
P = Q*10
print("P=", P)
R = 10

Npred = 5
Nsim = 100

# Objective def
target = vertcat(0, 0, 0, 0)
x = MX.sym('x', dx)
u = MX.sym('u', du)


func = Q*sumsqr(x[:]-target) + R*sumsqr(u[:])
f = Function('f', [x,u], [func], ['x','u'], ['objective'])
print("f(x,u)= ", f([1,1,1,1],[1,1]))

# MPC Problem

opti = Opti()

u = opti.variable(du, Npred)
x = opti.variable(dx, Npred+1)
# y = opti.variable(dy, Npred+1)

# print(x[:,1]*Q*x[:,1].T)

# u_tmp  = opti.variable(du, 1);
x_init =vertcat(15, 15, 0, 0);


opti.minimize(sumsqr(u) + sumsqr(x))

for k in range(Npred):
    opti.subject_to(x[:,k+1] == f(x[:,k], u[:,k]))

# opti.subject_to(umin <= u <= umax)
# opti.subject_to((x[:,Npred+1] ==  P*sumsqr(x[:,]-target)))
opti.subject_to(x[:,1] == x_init)

# Solver
opti.solver('sqpmethod', {'qpsol': 'qrqp'})
# opti.set_value(x_init, vertcat(15, 15, 0, 0))
sol = opti.solve()
print("sol= ", sol)