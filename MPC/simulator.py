import json
from MPC.optimizer import Optimizer
import numpy as np
import do_mpc
from casadi import *
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from MPC.utils import centroid
from cycler import cycler                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        

# TODO: make assert statements on attributes
# TODO: function to update,add,delete figure config in sim_struct.json: add "figname":{info} to file setFig(*args). If fig name already exists update else create new entry

class Simulator:   
    
    __location__ = os.path.realpath(
                        os.path.join(os.getcwd(), os.path.dirname(__file__))
                    )
    
    def __init__(self, optimizer: Optimizer) -> None:
        
        try:
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
            self.xinit = np.ones((optimizer.model.dx, optimizer.model.nb_agents))
            
            
            # Simulator results
            self.usim = DM(np.zeros((optimizer.model.du, optimizer.Nsim)));
            self.ysim = DM(np.zeros((optimizer.model.dy, optimizer.Nsim+1)));
            self.xsim = DM(np.zeros((optimizer.model.dx, optimizer.Nsim+1)));
            # print(optimizer.createPositionsVector())
            positions = optimizer.createPositionsVector()
            if positions is not None:
                self.targetsim = DM(positions)
            
            self.xsim[:,0] = self.xinit
        except:
            e = str(sys.exc_info()[0]) + ": " + str(sys.exc_info()[1])
            raise Exception("MPC.simulator.py.__init__():\n"+ str(e)) 
    
    def launchSimulation(self):
        
        try:
            # Initialize simulation
            self.simulator.x0 = self.xinit
            self.estimator.x0 = self.xinit
            self.optimizer.mpc.x0 = self.xinit
            
            self.optimizer.mpc.set_initial_guess()
            A, B = self.optimizer.model.getAgentSystem()[:2]
            
            x0 = self.xinit
            
            for k in range(self.optimizer.Nsim):
                u0 = self.optimizer.mpc.make_step(x0)
                y_next = self.simulator.make_step(u0)
                x0 = self.estimator.make_step(y_next)
                
                self.usim[:,k]   = u0
                self.xsim[:,k+1] = A@x0[:self.optimizer.model.dx] + B@u0[:self.optimizer.model.du]
        except:
            e = str(sys.exc_info()[0]) + ": " + str(sys.exc_info()[1])
            print("MPC.simulator.py.launchSimulation():\n"+ e) 
            
        return 0
    
    def show(self, _figname:str=None, _simStruct_path:str=None):
        
        try:
            titles, data = self.getSimStruct(path=_simStruct_path)
            
            # To plot a particular figure in sim_struct else plot all figures
            if _figname is not None:
                data = {_figname:data[_figname]}
                titles = [_figname]

            # Initialize figures
            nb_plot = len(data)
            fig = [None]*nb_plot
            ax  = [None]*nb_plot
        
        
            for i in range(nb_plot):
                plt.ion()
                fig[i] = plt.figure(i)
                ax[i] = fig[i].add_subplot()
                
                config = data[str(titles[i])]
                vars = config["variables"]
                print("[info]: plotting ", titles[i])
                
                fig[i].suptitle(titles[i])
                ax[i].set_xlabel(config['xlabel'])
                ax[i].set_ylabel(config['ylabel'])
                ax[i].set_xlim(config['xlim'])
                ax[i].set_ylim(config['ylim'])
                
                # Set obstacle if there is
                if config['has_obstacles']:
                    obstacles = self.optimizer.listOfObstacle
                    for x in range(len(obstacles)):
                        polygon = Polygon(obstacles[x])
                        ax[i].add_patch(polygon)
                        ax[i].annotate(str(x), xy=centroid(obstacles[x]), ha='center', va='center')
                
                plt.grid()
                # Plot at each step
                for k in range(self.optimizer.Nsim):
                    # plot each variable
                    for j in range(len(config["variables"])):
                        var = self.__toVar__(vars[j]) #get variable to plot
                        
                        if len(config["indexes"][j]) == 1:
                            kwargs = {'linefmt': config["colors"][j], 'markerfmt': config["markers"][j]} 
                            args = var[config["indexes"][j][0],k].full()
                        
                            ax[i].stem(k, args,**kwargs)
                            ax[i].legend(config["variables"])
                            
                            
                        else:
                            kwargs = {'color': config["colors"][j], 'marker': config["markers"][j],'linestyle': config["linestyles"][j]} 
                            args = [var[x, k] for x in config["indexes"][j] ]# set the variables to plot  
                            
                            ax[i].plot(*args, **kwargs)
                            ax[i].legend(config["variables"])
                            
                        fig[i].canvas.draw()
                        fig[i].canvas.flush_events()
                        
                plt.show()              
        except:
            e = str(sys.exc_info()[0]) + ": " + str(sys.exc_info()[1])
            print("MPC.simulator.py.show():\n"+ str(e)) 
            
        return 0
     
    def getSimStruct(self, path:str=None):
        titles = []
        try:
            file = open(os.path.join(self.__location__, "sim_struct.json"))
            if path is not None:
                file = open(path)
            data = json.load(file)
            titles = list(data.keys())
            # Close opend file
            file.close()
        except:
            e = str(sys.exc_info()[0]) + ": " + str(sys.exc_info()[1])
            raise Exception("MPC.simulator.py.__init__():\n"+ str(e)) 
        
        return titles, data
        
    def __toVar__(self, varname:str):
        try:
            var = None
            if varname == "xsim":
                var = self.xsim
            elif varname == "ysim":
                var = self.ysim
            elif varname == "usim":
                var = self.usim
            elif varname == "targetsim":
                var = self.targetsim.T
            else:
                pass
            
        except:
            e = str(sys.exc_info()[1])
            print("MPC.simulator.py.__toVar__():\n"+ str(e)) 
            
        return var
