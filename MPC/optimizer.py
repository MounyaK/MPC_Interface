import numpy as np
import do_mpc
from casadi import *
from MPC.model import Model
import polytope
from MPC.utils import centroid

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
            self.P = self.Q
            self.R = 10
            
            # default initial and target position
            # self.xinit = np.ones((self.model.dx, self.model.nb_agents))
            # self.listOfPositions.append((0, np.zeros((self.model.dx, self.model.nb_agents))))
        
        except:
            e = str(sys.exc_info()[0]) + ": " + str(sys.exc_info()[1])
            raise Exception("MPC.optimizer.__init__():\n"+str(e))
            
    def setup(self):
        try:
            self.mpc.setup()
        except:
            return "MPC.optimizer.setup(): Error setting up optimizer. Did you run all the steps correctly?"
        
        return 0
    
    def setObstacleConstraints(self):
         
        try:
            for i in range(len(self.listOfObstacle)):
                vertices = self.listOfObstacle[i]
                center = centroid(vertices)
                max_dist = self.maxDistance(vertices, center)
                print("center = ",center)
                for j in range(self.model.nb_agents):
                    x = self.model.model.x["x"+str(j)]
                    poly =  polytope.qhull(np.array(vertices))
                    print("Ax =", poly.A)
                    print("bx =", poly.b)
                    limit = -max_dist #- (1*max_dist)/100 # A marge of max(radius)+5%
                    self.mpc.set_nl_cons('obs_'+str(i)+'_constr_'+str(j), -sqrt((x[0]-SX(center)[0])**2 + (x[1]-SX(center)[1])**2), ub=limit)     
        
        except:
            e = str(sys.exc_info()[0]) + ": " + str(sys.exc_info()[1])
            raise Exception("MPC.optimizer.setObstacleConstraints():\n"+ str(e))
        
        return 0
    
    def setBoundsConstraints(self):
        try:
            
            for i in range(self.model.nb_agents):
            
                if self.umin is not None:    
                    self.mpc.bounds['lower','_u', "u"+str(i)] = self.umin
                if self.umax is not None:  
                    self.mpc.bounds['upper','_u', "u"+str(i)] = self.umax
                if self.xmin is not None:  
                    self.mpc.bounds['lower','_x', "x"+str(i)] = self.xmin
                if self.xmax is not None:  
                    self.mpc.bounds['upper','_x', "x"+str(i)] = self.xmax
            
        except:
            e = str(sys.exc_info()[0]) + ": " + str(sys.exc_info()[1])
            raise Exception("MPC.optimizer.setBoundsConstraints():\n"+ str(e)) 
        return 0
    
    def setTrajectory(self):
       
            
        try:
            tvp_template_mpc = self.mpc.get_tvp_template()
            Npred = self.__setup_mpc['n_horizon']
            positions = self.createPositionsVector()
            print("position= ", positions, "\nlength=", len(positions))
            
            # Set function which returns time-varying parameters
            def tvp_fun_mpc(t_now):
                for i in range(self.model.nb_agents):
                    for k in range(Npred):
                        tvp_template_mpc['_tvp', k, 'target'+str(i)] = positions[int(t_now+k)]
                            
                return tvp_template_mpc
            self.mpc.set_tvp_fun(tvp_fun_mpc)
                
        except:
            e = str(sys.exc_info()[0]) + ": " + str(sys.exc_info()[1])
            raise Exception("MPC.optimizer.setTrajectory(): Error setting up trajectory check positions vectors:\n"+ str(e)) 
        
        return 0
    
    def maxDistance(self, vertexes:list, centroid:list):
        try:
            max_dist = 0
            for i in range(len(vertexes)):
                dist = (abs(vertexes[i][0]-centroid[0])**2 + abs(vertexes[i][1]-centroid[1])**2)**0.5
                if dist > max_dist:
                    max_dist = dist
        
        except:
            return "MPC.optimizer.centroid(): Error finding center of polyhedrons."
        
        return max_dist
    
    def createPositionsVector(self) -> list:
        """ create list of instant t at which the targets are supposed to be reached in function of the number of targets and Nsim"""   
        try:
  
            lentgh = len(self.listOfPositions)
            print("positions= ", self.listOfPositions)
            print("len= ", lentgh)
            if lentgh > 1: #if there are targets
                time = [x for x in range(0,self.Nsim, int(self.Nsim/lentgh))]  
                current_t = time[0]
                next_t = time[1]
                positions = []
            
                for i in range(1,lentgh):
                    positions += [self.listOfPositions[i] for x in range(current_t,next_t)]  
                    
                    if i == lentgh-1:
                        next_t = self.__setup_mpc['n_horizon'] + self.Nsim
                    else:
                        next_t = time[i+1] 
                        
                    current_t = time[i]
                
                positions += [self.listOfPositions[i] for x in range(current_t,next_t)]
            elif lentgh == 1:
                positions = [self.listOfPositions[0] for x in range(self.__setup_mpc['n_horizon'] + self.Nsim)]
            else:
                positions = None     

        except:
            return "MPC.optimizer.createPositionsVector(): Error creating position vector for trajectory. Check ListOfPositions attribute is set correctly"        
        
        return positions 
    
    def setObjective(self):
        
        try:
            mterm = 0
            lterm = 0
            rterm = {}
            for i in range(self.model.nb_agents):
                mterm += mtimes(mtimes((self.model.model.x["x"+str(i)]-self.model.model.tvp["target"+str(i)]).T,self.Q), (self.model.model.x["x"+str(i)]-self.model.model.tvp["target"+str(i)]))
                lterm += mtimes(mtimes((self.model.model.x["x"+str(i)]-self.model.model.tvp["target"+str(i)]).T,self.P), (self.model.model.x["x"+str(i)]-self.model.model.tvp["target"+str(i)]))
                rterm["u"+str(i)] = self.R
            
            self.mpc.set_objective(mterm=mterm, lterm=lterm)
            self.mpc.set_rterm(
             **rterm
            )

            
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
    
    def getHorizon(self):
        return self.__setup_mpc['n_horizon'] 