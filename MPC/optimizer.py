import numpy as np
import do_mpc
from casadi import *
from MPC.model import Model
import pypoman
import polytope

# TODO: make assert statements on attributes
class Optimizer:
    # Simulation time
    Nsim = 100
    
    __setup_mpc = {
    'n_horizon': 5,
    't_step': 1,    # TODO: set position vector for every step
    'n_robust': 1,
    'store_full_solution': True,
    }
    
    # Constraints/Parameters
    umin = None
    umax = None
    xmin = None
    xmax = None
    
    # first state
    # xinit = None
    
    # Obstacle
    listOfObstacle = []
    
    # Trajectory
    listOfPositions = []    # position = tuple (time, array)
    
    
    def __init__(self, model:Model):
        
        try:
            self.model = model
            
            # set up mpc
            self.mpc = do_mpc.controller.MPC(self.model.model)
            
            # Set parameters
            self.mpc.set_param(**self.__setup_mpc)
            
            # Set tuning matrices
            self.Q = np.eye(model.dx)
            self.P = 10*self.Q
            self.R = 10
            
            # default initial and target position
            # self.xinit = np.ones((self.model.dx, self.model.nb_agents))
            self.listOfPositions.append((0, np.zeros((self.model.dx, self.model.nb_agents))))
        
        except:
            raise Exception("MPC.optimizer.__init__(): Error initializing optimizer. Check model instance")
            
    def setup(self):
        try:
            self.mpc.setup()
        except:
            return "MPC.optimizer.setup(): Error setting up optimizer. Did you run all the steps correctly?"
        
        return 0
    
    def setObstacleConstraints(self):
        
        try:
            for i in range(len(self.listOfObstacle)):
                poly = self.listOfObstacle[i]
                center = self.__Centroid__(pypoman.compute_polytope_vertices(poly.A, poly.b))
                print(center)
                for j in range(self.model.nb_agents):
                    x = self.model.model.x["x"+str(j)]
                    limit = -abs(np.amax(poly.b)) - (1*abs(np.amax(poly.b)))/100 # A marge of max(radius)+5%
                    print(limit)
                    self.mpc.set_nl_cons('obs_'+str(i)+'_constr_'+str(j), -sumsqr((x[:self.model.dy]-SX(center))**2), ub=limit)
            
        except:
            return "MPC.optimizer.setObstacleConstraints(): Error setting obstacle constraints. Check polyhedrons or model"
        
        return 0
    
    def setTrajectory(self):
        tvp_template_mpc = self.mpc.get_tvp_template()
        Npred = self.__setup_mpc['n_horizon']
        positions = self.createPositionsVector()
        
        # Set function which returns time-varying parameters
        def tvp_fun_mpc(t_now):
            for i in range(self.model.nb_agents):
                for k in range(Npred):
                    tvp_template_mpc['_tvp', k, 'target'+str(i)] = positions[int(t_now+k)]
                        
            return tvp_template_mpc
        self.mpc.set_tvp_fun(tvp_fun_mpc)
            
        # try:
        #     tvp_template_mpc = self.mpc.get_tvp_template()
        #     Npred = self.__setup_mpc['n_horizon']
        #     positions = self.createPositionsVector()
            
        #     # Set function which returns time-varying parameters
        #     def tvp_fun_mpc(t_now):
        #         for i in range(self.model.nb_agents):
        #             for k in range(Npred):
        #                 tvp_template_mpc['_tvp', k, 'target'+str(i)] = positions[int(t_now+k)]
                            
        #         return tvp_template_mpc
        #     self.mpc.set_tvp_fun(tvp_fun_mpc)
            
        # except:
        #     return "MPC.optimizer.setTrajectory(): Error setting up trajectory check positions vectors"
        
        return 0
    
    def __Centroid__(self, vertexes:list):
        try:
            _x_list = [vertex [0] for vertex in vertexes]
            _y_list = [vertex [1] for vertex in vertexes]
            _len = len(vertexes)
            _x = sum(_x_list) / _len
            _y = sum(_y_list) / _len
        
        except:
            return "MPC.optimizer.__Centroid__(): Error finding center of polyhedrons."
        
        return [_x, _y]
    
    def createPositionsVector(self) -> list:
        try:
            current_t = 0
            next_t = self.listOfPositions[0][0]
            lentgh = len(self.listOfPositions)
            positions = []
            
            for i in range(lentgh):
                positions += [self.listOfPositions[i][1] for x in range(current_t,next_t)]  
                
                if i == lentgh-1:
                    next_t = self.__setup_mpc['n_horizon'] + self.Nsim
                else:
                    next_t = self.listOfPositions[i+1][0] 
                    
                current_t = self.listOfPositions[i][0]
            
            positions += [self.listOfPositions[i][1] for x in range(current_t,next_t)]     
        
        except:
            return "MPC.optimizer.createPositionsVector(): Error creating position vector for trajectory. Check ListOfPositions attribute is set correctly"        
        
        return positions 
    
    def setObjective(self):
        
        try:
            mterm = 0
            lterm = 0
            for i in range(self.model.nb_agents):
                mterm += mtimes(mtimes((self.model.model.x["x"+str(i)]-self.model.model.tvp["target"+str(i)]).T,self.Q), (self.model.model.x["x"+str(i)]-self.model.model.tvp["target"+str(i)]))
                lterm += mtimes(mtimes((self.model.model.x["x"+str(i)]-self.model.model.tvp["target"+str(i)]).T,self.P), (self.model.model.x["x"+str(i)]-self.model.model.tvp["target"+str(i)]))
            self.mpc.set_objective(mterm=mterm, lterm=lterm)
            
        except:
            return "MPC.optimizer.setObjective(): Error setting Objective. Check model states, target or tuning matrices"
        
        return 0
        self.M = nb

    def getTuningMatrices(self):
        """getTuningMatrices(self) --> [self.Q, self.P, self.R]"""
        return [self.Q, self.P, self.R]
    
    def setHorizon(self, nb:int):
        self.__setup_mpc['n_horizon'] = nb
        self.mpc.set_param(**self.__setup_mpc)