from MPC.model import Model
import numpy as np

model = Model()

# set number of agents
model.nb_agents = 2
# model.A = np.zeros(4)
# model.var_u = []
# set single agent system
res = model.setAgentSystem(filepath="system.json")
# print(res)
# print(model.A)
# print(model.B)
# print(model.C)
# print(model.D)
# print(model.getSizes())

# setup model
res = model.setupModel()
# print(res)
# print(model.var_x)
# print(model.var_y)
# print(model.var_u)
# print(model.var_target)
# print(model.model.x.keys())

print(res)
print(np.shape(model.X))
print(model.X)
print(np.shape(model.An))
print(model.An)
print(np.shape(model.model.aux['X_next']))
print(model.model.aux['X_next'])