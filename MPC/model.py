from typing import Any
import numpy as np
import do_mpc
from casadi import *
import json

class Model:
    """
        Setup MPC model:\n
        Number of Agents\n
        System matrices: A, B, C, D\n
        Matrices sizes:dx, du, dy\n
    """
    # Model Matrices
    nb_agents = 1
    
    # An = np.zeros(4)
    # Bn = np.zeros((4,2))
    # Cn = np.zeros((2,4))
    # Dn = np.zeros(2)
    
    A = np.zeros(4)
    B = np.zeros((4,2))
    C = np.zeros((2,4))
    D = np.zeros(2)
    
    # Sizes
    dx, du = np.shape(B)
    dy = np.size(C, 0)
    
    # Optimizer variables
    var_x = []
    var_y = []
    var_u = []
    var_target = []
    
    X = None
    Y = None
    U = None
    Target = None
    
    
    def __init__(self):
        model_type = 'discrete' # either 'discrete' or 'continuous'
        self.model = do_mpc.model.Model(model_type)

    def __setattr__(self, __name: str, __value: Any) -> None:
        if __name in ('X', 'Y', 'U', 'Target', 'var_x', 'var_y', 'var_u', 'var_target', 'A', 'B', 'C', 'D'):
            raise AttributeError("'{}' is read-only. Use dedicated functions to set a new value".format(__name))
        # return super().__getattribute__(self, __name)
   
    def setupModel(self):
        # Set the inputs, states and parameters for all agents
        res1 = self.setModelVariables()
        
        # Set x_next, y_next symbolic expressions
        res2 = self.setModelExpressions()
        
        # Set right hand equation for all agents
        res3 = self.setModelRhs()
        
        try:
            self.model.setup()
        except:
            return "MPC.model.setupModel(): Error setting up model\nMPC.model.setAgentStruct(): "+str(res1)+ "\nMPC.model.setModelExpressions()"+ str(res2)+"\nMPC.model.setModelRhs(): "+str(res3)
        
        return 0
        
    def setModelVariables(self):
        
        try:
            for i in range(self.nb_agents):
                # Set symbolic name for variables
                x_name = 'x' + str(i)
                u_name = 'u' + str(i)
                y_name = 'y' + str(i)
                target_name = 'target' + str(i)
                
                # States struct (optimization variables):
                self.var_x.append(self.model.set_variable(var_type='_x', var_name=x_name, shape=(self.dx,1)))
                self.var_y.append(self.model.set_variable(var_type='_x', var_name=y_name, shape=(self.dy,1)))
                # Input struct (optimization variables):
                self.var_u.append(self.model.set_variable(var_type='_u', var_name=u_name, shape=(self.du,1)))
                # parameters
                self.var_target.append(self.model.set_variable(var_type='_tvp', var_name=target_name, shape=(self.dx,1)))
            
        except:
            return "MPC.optimizerModel.setModelVariables(): Error setting optimizer agents states, inputs and parameters structure"
        
        return 0
            
    def setGlobalModelVariables(self):
        """setGlobalModelVariables(self) --> [X, Y, U, Target]"""
        try:
            isNotSingleton = len(self.var_x) > 1 and len(self.var_y) > 1 and len(self.var_u) > 1 and len(self.var_target) > 1
            if isNotSingleton:
                self.X, self.Y, self.U, self.Target = [horzcat(*self.var_x), horzcat(*self.var_y), horzcat(*self.var_u), horzcat(*self.var_target)] 
            else:
                self.X, self.Y, self.U, self.Target = [self.var_x[0], self.var_y[0], self.var_u[0], self.var_target[0]]
        except:
            return "MPC.optimizerModel.setGlobalModelVariables(): Error setting modelVariables, check var_x, var_y, var_u and var_target"
        return 0
    
    def setModelExpressions(self):
        # res1 = self.setGlobalSystem()
        res2 = self.setGlobalModelVariables()
        
        try:
            self.model.set_expression('X_next', self.A@self.X + self.B@self.U)
            self.model.set_expression('Y_next', self.C@self.X + self.D@self.U)
            
        except:
            return "MPC.optimizerModel.setModelExpressions(): Error setting expressions\nMPC.optimizerModel.setGlobalSystem(): "+str(res1)+"\nMPC.optimizerModel.setGlobalModelVariables(): "+str(res2)
        
        return 0
    
    def setModelRhs(self):
        try:
            for i in range(self.nb_agents):
                self.model.set_rhs("x"+str(i), self.A@self.model.x["x"+str(i)] + self.B@self.model.u["u"+str(i)])
                self.model.set_rhs("y"+str(i), self.C@self.model.x["x"+str(i)] + self.D@self.model.u["u"+str(i)])
            
        except:
            return "MPC.optimizerModel.setModelRhs(): Error setting model rhs"
        
        return 0
  
    def setAgentSystem(self, filepath:str=None, matrixList:list[np.array]=None):
        """ setSystem(self, file=None, matrixList=None)
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
        file = None
        if filepath is not None and  matrixList is None:
            try:
                # Get json file
                file = open(filepath)
                data = json.load(file)
                
                # Construct  System matrice
                self.A = np.array(data['A'])
                self.B = np.array(data['B'])
                self.C = np.array(data['C'])
                self.D = np.array(data['D'])
                
                # update size parameter
                self.dx, self.du = np.shape(self.B)
                self.dy = np.size(self.C, 0)
                    
            except:
                return "MPC.optimizerModel.setSystem(): Error in file input processing"
            
                
        elif filepath is None and matrixList is not None:
            try:
                self.A, self.B, self.C, self.D = matrixList
                # update size parameter
                self.dx, self.du = np.shape(self.B)
                self.dy = np.size(self.C, 0)
            except:
                return "MPC.optimizerModel.setModel(): Error in matrix list input processing"
        else:
            return "MPC.optimizerModel.setSystem(): Error cannot have input file and input matrix list together"
        
        return 0
  
    # def setGlobalSystem(self):
    #     try:
    #         # set mpc System according to number of agents
    #         self.An = np.concatenate(tuple([self.A for i in range(self.nb_agents  )]), axis=0)
    #         self.Bn = np.concatenate(tuple([self.B for i in range(self.nb_agents  )]), axis=0)
    #         self.Cn = np.concatenate(tuple([self.C for i in range(self.nb_agents  )]), axis=0)
    #         self.Dn = np.concatenate(tuple([self.D for i in range(self.nb_agents  )]), axis=0)
    #     except:
    #         return "MPC.optimizerModel.setGlobalSystem(): Error setting global matrices"
    #     return 0  
    
    
    def getAgentSystem(self):
        """getModel(self) --> [self.A, self.B, self.C, self.D ]"""
        return [self.A, self.B, self.C, self.D ]
    
    def getSizes(self):
        """getSizes(self) --> [self.dx, self.du, self.dy]"""
        return [self.dx, self.du, self.dy]