"""
##Model:
    - number of agents
        setNumberOfAgents(nb)
    - matrices A, B, C and D:
        getMatrices(file) => [A, B, C, D]
    - Tuning Matrices Q, P, R
        update(Q, P, R) **visually
    - polypes obstacles
        drawPolytope(fig) => poly
        listOfPoly<poly>
    - Constraints
        setBounds(umin, umax, xmin, xmax) **Optional
        setObstacleConstraints(listOfPoly)
    - Other parameters
        Npred, Nsim
""" 
import numpy as np
import do_mpc
import json
import utils.fileManagement as utils

class Optimizer:
    
    model_type = 'discrete' # either 'discrete' or 'continuous'
    model = do_mpc.model.Model(model_type)
    Npred = 5
    setup_mpc = {
    'n_horizon': Npred,
    't_step': 0.5,
    'n_robust': 1,
    'store_full_solution': True,
    }
    
    # Model Matrices
    nb_agents = 1
    
    An = np.zeros(4)
    Bn = np.zeros((4,2))
    Cn = np.zeros((2,4))
    Dn = np.zeros(2)
    
    A = np.zeros(4)
    B = np.zeros((4,2))
    C = np.zeros((2,4))
    D = np.zeros(2)
    
    # Sizes
    dx, du = np.shape(Bn)
    dy = np.size(Cn, 0)
    
    # Tuning Matrices
    Q = np.zeros(dx)
    P = np.zeros(dx)
    R = 10
    
    # Constraints/Parameters
    umin = None
    umax = None
    xmin = None
    xmax = None
    
    M = 1000
    x0 = np.zeros((dx,1))
    
    # Obstacle
    listOfObstacle = []
    
    def __init__(self):
        pass
    
    def setAgentModel(self, file=None, matrixList=None):
        """ setModel(self, file=None, matrixList=None)
            -->  0   : ok
            --> msg  : error in processing
            
            - Example of file:
            {
                'A':[
                    [1, 1, 1, 1],
                    [1, 1, 1, 1],
                    [1, 1, 1, 1],
                    [1, 1, 1, 1],  
                ],
                
                'B':[   
                    [0, 0], 
                    [0, 0], 
                    [0.5, 0], 
                    [0, 0.5] 
                ]
                
                (...)
            }
            
            - Ecample of matrix list:
            list = [np.array(4), np.array(4), .., ..]

        """
        if file is not None and  matrixList is None:
            try:
                # Get json file
                filepath = utils.getFile()
                file = open(filepath)
                data = json.load(file)
                
                # Construct  Model matrice
                self.A = data['A']
                self.B = data['B']
                self.C = data['C']
                self.D = data['D']
                    
            except:
                return "MPC.optimizerModel.setModel(): Error in file input processing"
                
        elif file is None and matrixList is not None:
            try:
                self.A, self.B, self.C, self.D = matrixList
            except:
                return "MPC.optimizerModel.setModel(): Error in matrix list input processing"
        else:
            return "MPC.optimizerModel.setModel(): Error cannot have input file and input matrix list together"
        return 0
  
    def setGlobalModel(self):
        # set mpc model according to number of agents
        self.An = np.concatenate(tuple([self.A for i in range(self.nb_agents  )]), axis=0)
        self.Bn = np.concatenate(tuple([self.B for i in range(self.nb_agents  )]), axis=0)
        self.Cn = np.concatenate(tuple([self.C for i in range(self.nb_agents  )]), axis=0)
        self.Dn = np.concatenate(tuple([self.D for i in range(self.nb_agents  )]), axis=0)
        
        # update size parameter
        self.dx, self.du = np.shape(self.Bn)
        self.dy = np.size(self.Cn, 0)
  
    def setNbOfAgents(self, nb):
        self.nb_agents = nb
    
    def setQ(self, array):
        self.Q = array
    
    def setQ(self, array):
        self.P = array
    
    def setQ(self, nb):
        self.R = nb
    
    def setUmin(self, nb):
        self.umin = nb
    
    def setUmax(self, nb):
        self.umax = nb
    
    def setXmin(self, nb):
        self.xmin = nb
    
    def setUmax(self, nb):
        self.umax = nb

    def setM(self, nb):
        self.M = nb

    def getModel(self):
        """getModel(self) --> [self.A, self.B, self.C, self.D ]"""
        return [self.A, self.B, self.C, self.D ]
    
    def getTuningMatrices(self):
        """getTuningMatrices(self) --> [self.Q, self.P, self.R]"""
        return [self.Q, self.P, self.R]
    
    def getObstacles(self):
        """getObstacles(self) --> listOfObstacle<poly>"""
        return self.listOfObstacle
    
    def getSizes(self):
        """getSizes(self) --> [self.dx, self.du, self.dy]"""
        return [self.dx, self.du, self.dy]
    
    def getNbOfAgents(self):
        """getNbOfAgents(self) --> self.nb_agents"""
        return self.nb_agents
    
    def getM(self):
        """getM(self) --> self.M"""
        return self.M
    
    def getX0(self):
        """getX0(self) --> self.x0"""
        return self.x0