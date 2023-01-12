import time
from MPC.model import Model
import numpy as np
from MPC.optimizer import Optimizer
import polytope as poly
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib.animation import FuncAnimation, ImageMagickWriter, FFMpegFileWriter, FileMovieWriter
import polytope as poly
from matplotlib.patches import Polygon
from MPC.simulator import Simulator
import pypoman

model = Model()

""" Set number of agents"""
model.nb_agents = 1
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

optimizer.setHorizon(20)


"""Set Objective"""
res = optimizer.setObjective()
print(res)


"""Set Obstacle Constraints"""
# Polyhedron definition
Ax = np.array([ [-1, 0],
                [1, 0], 
                [0, -1], 
                [0, 1]
            ])

bx = 0.5*np.ones((4,1))
obs = poly.Polytope(Ax, bx)
optimizer.listOfObstacle.append(pypoman.compute_polytope_vertices(obs.A, obs.b))
print(optimizer.listOfObstacle)

# Set constraints
optimizer.umin = -2
optimizer.umax = 2
optimizer.xmin = -10
optimizer.xmax = 10

res = optimizer.setObstacleConstraints()
print(res)
res = optimizer.setBoundsConstraints()
# res = optimizer.set
print(res)

# Set Trajectory
optimizer.listOfPositions = [np.array([4, -2, 0, 0]), np.array([-3, 3, 0, 0]), np.array([5, 2, 0, 0]), np.array([-4, 5, 0, 0])]#, np.array([4, 1, 0, 0]),np.array([-2, 0, 0, 0]), np.array([4, 0, 0, 0]) ] 
# optimizer.listOfPositions = np.sort(optimizer.listOfPositions)
print("sorted targets= ",optimizer.listOfPositions)
res = optimizer.setTrajectory()
print(res)

optimizer.setup()

"""Set up simulator"""
# optimizer.xinit = np.array([2,2,0,0,0,0])
simulator = Simulator(optimizer)
simulator.xinit = np.array([5, 5, 0, 0])
# simulate
res = simulator.launchSimulation()
# show plot
# res =  simulator.show("Position")
res =  simulator.show("Tracking")
# res =  simulator.show("Vx(t)")
# res =  simulator.show("Vy(t)")

# for k in range(optimizer.Nsim):
#     print(simulator.xsim[:,k])
#     ax[0].scatter(simulator.xsim[0,k].full(), simulator.xsim[1,k].full(), color='blue') 
#     ax[1].stem(k, simulator.xsim[2,k].full(), '--', use_line_collection = True)
#     ax[2].stem(k, simulator.xsim[3,k].full(), '--', use_line_collection = True)
    
#     fig.canvas.draw()
#     fig.canvas.flush_events()
