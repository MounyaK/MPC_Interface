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

# set r term manually (no other ways for now)
optimizer.mpc.set_rterm(
    u0 = optimizer.R
)

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
optimizer.umin = -100
optimizer.umax = 100
optimizer.xmin = -100
optimizer.xmax = 100

res = optimizer.setObstacleConstraints()
# res = optimizer.set
print(res)

# Set Trajectory
optimizer.listOfPositions = [(15,np.array([5, 5, 0, 0])), (30,np.array([-3, 0, 0, 0])), (50,np.array([4, 1, 0, 0])) ] 
res = optimizer.setTrajectory()
print(res)

optimizer.setup()

"""Set up simulator"""
# optimizer.xinit = np.array([2,2,0,0,0,0])
simulator = Simulator(optimizer)
simulator.xinit = np.array([2, 2, 0, 0, 2, 2])
# simulate
res = simulator.launchSimulation()
# show plot
res =  simulator.show("Position")
# plt.show()
# print("finished")
# plt.ion()
# # Plot parameters
# fig, ax = plt.subplots(3, sharex=False, figsize=(16,9))
# fig.align_ylabels()

# ax[0].set_ylabel('x2')
# ax[0].set_xlabel('x1')
# ax[0].set_xlim(-float(np.amax(simulator.xsim[1:].full())), float(np.amax(simulator.xsim[1:].full())+1))
# ax[0].set_ylim(-float(np.amax(simulator.xsim[2:].full())), float(np.amax(simulator.xsim[2:].full())+1))

# ax[1].set_xlabel('time [s]')
# ax[1].set_ylabel('Vx(t)')
# ax[1].set_xlim(0.0, float(optimizer.Nsim))
# ax[1].set_ylim(float(np.amin(simulator.xsim[2:].full())-1), float(np.amax(simulator.xsim[2,:].full())+1))

# ax[2].set_xlabel('time [s]')
# ax[2].set_ylabel('Vy(t)')
# ax[2].set_xlim(0.0, float(optimizer.Nsim))
# ax[2].set_ylim(float(np.amin(simulator.xsim[3,:].full())-1), float(np.amax(simulator.xsim[3,:].full())+1))

# polygon = Polygon(pypoman.compute_polytope_vertices(optimizer.listOfObstacle[0].A, optimizer.listOfObstacle[0].b))
# ax[0].add_patch(polygon)

# print(simulator.xsim)
# # ax[0].plot_polygon(vertices)
# for k in range(optimizer.Nsim):
#     print(simulator.xsim[:,k])
#     ax[0].scatter(simulator.xsim[0,k].full(), simulator.xsim[1,k].full(), color='blue') 
#     ax[1].stem(k, simulator.xsim[2,k].full(), '--', use_line_collection = True)
#     ax[2].stem(k, simulator.xsim[3,k].full(), '--', use_line_collection = True)
    
#     fig.canvas.draw()
#     fig.canvas.flush_events()
