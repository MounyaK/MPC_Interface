from MPC.optimizer import Optimizer
import numpy as np
import do_mpc
from casadi import *
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib.animation import FuncAnimation, ImageMagickWriter, FFMpegFileWriter, FileMovieWriter

# TODO: make assert statements on attributes
# TODO: function to update or add figure config in sim_struct.json

# sim_struct = dict.fromkeys(['nb_figure','ball']) # Describe the variables and graphique to add to the simulator

class Simulator:   
    
    def __init__(self, optimizer: Optimizer) -> None:
        self.optimizer = optimizer
        
        # Set up simulator
        self.simulator = do_mpc.simulator.Simulator(optimizer.model.model)
        
        # Instead of supplying a dict with the splat operator (**), as with the optimizer.set_param(),
        # we can also use keywords (and call the method multiple times, if necessary):
        self.simulator.set_param(t_step = 0.1)
        
        # Set function which returns time-varying parameters
        tvp_template_sim = self.simulator.get_tvp_template()
        def tvp_fun_sim(t_now):
                return tvp_template_sim
        self.simulator.set_tvp_fun(tvp_fun_sim)
        
        self.simulator.setup()
        self.estimator = do_mpc.estimator.StateFeedback(optimizer.model.model)

        # First state
        self.xinit = np.ones((optimizer.model.dx + optimizer.model.dy, optimizer.model.nb_agents))
         
        
        # Simulator results
        self.usim = DM(np.zeros((optimizer.model.du, optimizer.Nsim)));
        self.ysim = DM(np.zeros((optimizer.model.dy, optimizer.Nsim+1)));
        self.xsim = DM(np.zeros((optimizer.model.dx, optimizer.Nsim+1)));
        
        self.xsim[:,0] = self.xinit[:optimizer.model.dx]

    def launchSimulation(self):
        # Initialize simulation
        self.simulator.x0 = self.xinit
        self.estimator.x0 = self.xinit
        self.optimizer.mpc.x0 = self.xinit
        
        self.optimizer.mpc.set_initial_guess()
        A, B, C, D = self.optimizer.model.getAgentSystem()
        
        x0 = self.xinit
        print(x0)
        for k in range(self.optimizer.Nsim):
            u0 = self.optimizer.mpc.make_step(x0)
            y_next = self.simulator.make_step(u0)
            x0 = self.estimator.make_step(y_next)
            
            self.usim[:,k]   = u0
            self.xsim[:,k+1] = A@x0[:self.optimizer.model.dx] + B@u0[:self.optimizer.model.du]

    # TODO: a fig for each plot to draw
    def show():
        pass