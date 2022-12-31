from MPC.model import Model
import numpy as np
from MPC.optimizer import Optimizer
import polytope as poly

model = Model()

""" Set number of agents"""
model.nb_agents = 2
# model.A = np.zeros(4)
# model.var_u = []
# set single agent system
res = model.setAgentSystem(filepath="system.json")
print(res)
A, B, C, D = model.getAgentSystem()
print("A='{}'\nB='{}'\nC='{}'\nD='{}'\n".format(A, B, C, D) )
# print(model.A)
# print(model.B)
# print(model.C)
# print(model.D)
# print(model.getSizes())

"""setup model"""
res = model.setupModel()
print(res)
# print(model.var_x)
# print(model.var_y)
# print(model.var_u)
# print(model.var_target)
print(model.model.x.keys())

# print(res)
# print(np.shape(model.X))
# print(model.X)
# print(np.shape(model.An))
# print(model.An)
print(np.shape(model.model.aux['X_next']))
print(model.model.aux['X_next'])

print(model.model)

"""Setup Optimizer"""
optimizer = Optimizer(model)

"""Set Objective"""
res = optimizer.setObjective()
print(res)

optimizer.mpc.set_rterm(
    u0 = optimizer.R,
    u1 = optimizer.R
)
# set r term manually (no other ways)

"""Set Obstacle Constraints"""
# Polyhedron definition
Ax = np.array([ [-1, 0],
                [1, 0], 
                [0, -1], 
                [0, 1]
            ])

bx = 0.4*np.ones((4,1))
obs = poly.Polytope(Ax, bx)
optimizer.listOfObstacle.append(obs)
print(optimizer.listOfObstacle)

# Set constraints
optimizer.umin = -10
optimizer.umax = 10
optimizer.xmin = -10;
optimizer.xmax = 10;

res = optimizer.setObstacleConstraints()
print(res)

# Set Trajectory
optimizer.listOfObstacle = [(15,np.array([5, 5, 0, 0])), (30,np.array([-3, 0, 0, 0])), (50,np.array([4, 1, 0, 0])) ] 
res = optimizer.setTrajectory()
print(res)