import numpy as np
import do_mpc
from casadi import *
from model import Model
import pypoman

class optimizer:
    # Simulation time
    Nsim = 100
    
    setup_mpc = {
    'n_horizon': 5,
    't_step': 0.5,
    'n_robust': 1,
    'store_full_solution': True,
    }
    
    # Constraints/Parameters
    umin = None
    umax = None
    xmin = None
    xmax = None
    
    # first state
    xinit = None
    
    # Obstacle
    listOfObstacle = []
    
    # Trajectory
    listOfPositions = [] # position = tuple (time, array)
    
    def __init__(self, model:Model):
        
        self.model = model
        # set up mpc
        self.mpc = do_mpc.controller.MPC(model)
        
        # Set parameters
        self.mpc.set_param(**self.setup_mpc)
        
        # Set tuning matrices
        self.Q = np.array(model.dx)
        self.P = 10*self.Q
        self.R = 10
        
        self.xinit = np.zeros((self.model.dx, self.model.nb_agents))
       
    def setObstacleConstraints(self):
        try:
            for i in range(len(self.listOfObstacle)):
                poly = self.listOfObstacle[i]
                center = self.__Centroid__(poly, pypoman.compute_polytope_vertices(poly.A, poly.B))
                for j in range(self.model.nb_agents):
                    y = self.model.var_y[j]
                    limit = -abs(np.amax(poly.B)) + (5*abs(np.amax(poly.B)))/100 # A marge of max(radius)+5%
                    self.mpc.set_nl_cons('obs_'+str(i)+'_constr_'+str(j), -sumsqr((y-center)**2), ub=limit)
            
        except:
            return "MPC.optimizer.setObstacleConstraints(): Error setting obstacle constraints. Check polyhedrons or model"
        
        return 0
    
    def setTrajectory(self):
        tvp_template_mpc = self.mpc.get_tvp_template()
        Npred = self.setup_mpc['n_horizon']
        positions = self.__createPositionsVector__()
        # Set function which returns time-varying parameters
        def tvp_fun_mpc(t_now):
            positions
            for k in range(Npred):
                print(t_now+k)
                tvp_template_mpc['_tvp', k, 'target'] = positions[int(t_now+k)]
                    
            return tvp_template_mpc
        self.mpc.set_tvp_fun(tvp_fun_mpc)
    
    def __Centroid__(self, vertexes:list):
        _x_list = [vertex [0] for vertex in vertexes]
        _y_list = [vertex [1] for vertex in vertexes]
        _len = len(vertexes)
        _x = sum(_x_list) / _len
        _y = sum(_y_list) / _len
        return [_x, _y]
    
    def __createPositionsVector__(self) -> list:
        current_t = 0
        next_t = self.listOfPositions[0][0]
        lentgh = len(self.listOfPositions)
        positions = []
        
        for i in range(lentgh):
            positions += [self.listOfPositions[i][1] for x in range(current_t,next_t)]  
              
            if i == lentgh-1:
                next_t = self.setup_mpc['n_horizon'] + self.Nsim
            else:
                next_t = self.listOfPositions[i+1][0] 
                
            current_t = self.listOfPositions[i][0]
        
        positions += [self.listOfPositions[i][1] for x in range(current_t,next_t)]             
        return positions 
    
    def setObjective(self):
        try:
            mterm = mtimes(mtimes((self.model.X-self.model.Target).T,self.Q), (self.model.X-self.model.Target))
            lterm = mtimes(mtimes((self.model.X-self.model.Target).T,self.P), (self.model.X-self.model.Target))
            self.mpc.set_objective(mterm=mterm, lterm=lterm)
            
        except:
            return "MPC.optimizer.setObjective(): Error setting Objective. Check model states, target or tuning matrices"
        
        return 0
        self.M = nb

    def getTuningMatrices(self):
        """getTuningMatrices(self) --> [self.Q, self.P, self.R]"""
        return [self.Q, self.P, self.R]