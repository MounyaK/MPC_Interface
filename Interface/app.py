
# from MPC.model import Model
from pathlib import Path
from re import split
from Interface.ui_Graph import Ui_GraphDialog
from Interface.ui_addElement import Ui_Dialog
from MPC.model import Model
from MPC.optimizer import Optimizer
from MPC.simulator import Simulator
from ui_MainWindow import Ui_MainWindow
import sys
from PyQt6.QtCore import QFileInfo
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
from PyQt6.QtWidgets import QApplication, QMainWindow, QMessageBox, QFileDialog, QDialog
from PyQt6 import QtGui
import os
import numpy as np
import polytope as poly
import matplotlib.pyplot as plt
import polytope as poly
from matplotlib.patches import Polygon
import pypoman
from MPC.utils import centroid
from casadi import horzcat

# Subclass QMainWindow to customize your application's main window
class MainWindow(QMainWindow):
    
    # MPC objects
    model     = None
    optimizer = None
    simulator = None
    
    obstacles = []
    targets = None
    coords = []
    
    simStruct_path = os.path.join(os.getcwd(),"Interface", "ressources", "sim_struct.json")
    
    def __init__(self):
        # mainWindow = QMainWindow()
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
 
        # Initialize some ui vars
        self.ui.nbAgents.setPlainText("1")
        # self.ui.layoutChanged.emit()
        
        # Connect buttons to functions
        self.ui.launchButton.clicked.connect(self.onLaunchButtonClicked)
        self.ui.setSysButton.clicked.connect(self.onSetSysButton) 
        self.ui.addTargetButton.clicked.connect(self.onAddtargetButtonClicked)
        self.ui.addObsButton.clicked.connect(self.onaddObsButtonClicked)
        self.ui.delTargetButton.clicked.connect(self.onDelTargetButtonClicked)
        self.ui.delObsButton.clicked.connect(self.onDelObsButtonClicked)
        
        # Actions
        self.ui.actionLoadDefault.triggered.connect(self.setDefaultSystem)
       
    def ui_message(self, msg:str, type:str="info"):
        dlg = QMessageBox(self)
        dlg.setWindowTitle(type)
        dlg.setText(msg)
        button = dlg.exec()

        if button == QMessageBox.StandardButton.Ok:
            
            dlg.close()
    
    def onLaunchButtonClicked(self):
        
        self.targets = self.optimizer.listOfPositions
        self.obstacles = self.optimizer.listOfObstacle
        # for x in range(len(self.optimizer.listOfObstacle)):
        #     self.obstacles[self.ui.obstacles.itemText(x)] = self.optimizer.listOfObstacle[x]
        
        self.__setOptimizer__()
        self.ui.progressBar.setValue(50)
        self.simulator.launchSimulation()
        self.ui.progressBar.setValue(100)
        self.simulator.show(self.ui.plots.currentText(), self.simStruct_path) 
        self.ui.progressBar.setValue(0)  
        self.__setPlotList__()
        
    def onSetSysButton(self):
        try:
            dlg = QFileDialog()
            # dlg.setLayout(QVBoxLayout()) #align content vertically
            fileName = dlg.getOpenFileName(self, 'Choose system file', '', 'System Files (*.json)')
            
            if fileName[0]:
                print(fileName[0])
                self.model = Model()
                self.model.setAgentSystem(filepath=fileName[0])
                self.ui.currentModel.setMarkdown(Path(fileName[0]).name) # Update current model ui 
                self.ui_message("Operation Succeeded")
        except:
            self.ui_message("Operation Failed", "Error")
            e = str(sys.exc_info()[0]) + ": " + str(sys.exc_info()[1])
            print(e)
    
    def setDefaultSystem(self):
        
        try:
            self.model = Model()
            """ Set number of agents"""
            self.model.nb_agents = 1
            default_sys = os.path.join(os.getcwd(),"Interface", "ressources", "system.json")
            
            self.model.setAgentSystem(filepath=default_sys)
            self.ui.currentModel.setMarkdown("system.json") # Update current model ui
            A, B, C, D = self.model.getAgentSystem()
            print("Current Model:\nA='{}'\nB='{}'\nC='{}'\nD='{}'\n".format(A, B, C, D) )
            
            """setup model"""
            self.model.setupModel()
        
            """Setup Optimizer"""
            self.optimizer = Optimizer(self.model)

            self.optimizer.setHorizon(20)
            self.ui.Horizon.setPlainText(str(self.optimizer.getHorizon()))

            """Set Objective"""
            self.optimizer.setObjective()
            
            """Set Obstacle Constraints"""
            # Polyhedron definition
            Ax = np.array([ [-1, 0],
                            [1, 0], 
                            [0, -1], 
                            [0, 1]
                        ])
            bx = 0.4*np.ones((4,1))
            obs = poly.Polytope(Ax, bx)
            obs = pypoman.compute_polytope_vertices(obs.A, obs.b)
            self.optimizer.listOfObstacle.append(obs)
            self.ui.obstacles.addItem("obstacle_0")
            self.obstacles.append(obs) # use index for delete
            
            # Set constraints
            self.optimizer.umin = -2
            self.ui.umin.setPlainText(str(self.optimizer.umin))
            
            self.optimizer.umax = 2
            self.ui.umax.setPlainText(str(self.optimizer.umax))
            
            self.optimizer.xmin = -10
            self.ui.xmin.setPlainText(str(self.optimizer.xmin))
            
            self.optimizer.xmax = 10
            self.ui.xmax.setPlainText(str(self.optimizer.xmax))

            self.optimizer.setObstacleConstraints()
            self.optimizer.setBoundsConstraints()

            self.ui.Nsim.setPlainText(str(self.optimizer.Nsim))

            # Set Trajectory
            self.optimizer.listOfPositions = [np.array([5, 5, 0, 0]), np.array([-3, 0, 0, 0]), np.array([4, 1, 0, 0]) ] 
            self.optimizer.setTrajectory()
            self.ui.targets.addItems(["target_"+str(x) for x in range(len(self.optimizer.listOfPositions))])

            self.optimizer.setup()

            """Set up simulator"""
            self.simulator = Simulator(self.optimizer)
            self.simulator.xinit = np.array([2, 2, 0, 0])
            self.ui.xinit.setPlainText("2 2 0 0")
            
            self.__setPlotList__()
            
            self.ui_message("Initialized simulator with default system.", "Info")
            
        except:
            self.ui_message("Couldn't initialize default model.", "Error")
            e = str(sys.exc_info()[0]) + ": " + str(sys.exc_info()[1])
            print(e)

    def onAddtargetButtonClicked(self):
         
        def onclick(event):
            # global ix, iy, fig, ax
            ix, iy = event.xdata, event.ydata
           
            # assign global variable to access outside of function
            # global coords
            self.coords = [ix, iy] + [0.0 for x in range(self.model.dx - 2)]
            ax.plot(ix, iy, 'b+')
            self.optimizer.listOfPositions.append( np.array(self.coords) )
            print("new target:", np.array(self.coords))
            self.ui.targets.addItem("target_"+str(len(self.optimizer.listOfPositions)-1))
            self.coords = []
            fig.canvas.draw()
            fig.canvas.flush_events()
    
        # Style plot
        fig = plt.figure(1)
        ax = fig.add_subplot(111)

        data = self.simulator.getSimStruct()[1][self.ui.plots.currentText()]
        xlim, ylim = [data["xlim"], data["ylim"]]
        ax.set_ylim(xlim)
        ax.set_xlim(ylim)
        # ax.set_xlabel("Press enter to create a new Polygon")
        ax.set_title("Add new targets")

        self.__plotEnv__(ax)
        # Call click func
        fig.canvas.mpl_connect('button_press_event', onclick)
        # fig.canvas.mpl_connect('key_press_event', onpress)

        plt.grid()
        plt.show()
                      
    def onaddObsButtonClicked(self):
        # coords = []
        # callback funcs
        def onclick(event):
            # global ix, iy, fig, ax
            ix, iy = event.xdata, event.ydata
            
            # print 'x = %d, y = %d'%(
            #     ix, iy)

            # assign global variable to access outside of function
            # global coords
            self.coords.append((ix, iy))
            ax.plot(ix, iy, 'b+')
            fig.canvas.draw()
            fig.canvas.flush_events()

        def onpress(event):
            # global coords
            if event.key == 'enter':
                # Draw polygon on figure
                polygon = Polygon(self.coords)
                ax.add_patch(polygon)
                fig.canvas.draw()
                fig.canvas.flush_events()
                # Update obstacles list in optimizer instance and ui
                self.optimizer.listOfObstacle.append(self.coords)
                self.ui.obstacles.addItem("obstacle_"+str(len(self.optimizer.listOfObstacle)-1))
                print(self.coords)
                print("updated obs = ",self.optimizer.listOfObstacle )
                self.coords = [] #reset coordinates list

        # Style plot
        fig = plt.figure(1)
        ax = fig.add_subplot(111)

        data = self.simulator.getSimStruct()[1][self.ui.plots.currentText()]
        xlim, ylim = [data["xlim"], data["ylim"]]
        ax.set_ylim(xlim)
        ax.set_xlim(ylim)
        ax.set_xlabel("Press enter to create a new Polygon")
        ax.set_title("Add new obstacles")

        self.__plotEnv__(ax)
        # Call click func
        fig.canvas.mpl_connect('button_press_event', onclick)
        fig.canvas.mpl_connect('key_press_event', onpress)

        plt.grid()
        plt.show()
    
    def onDelTargetButtonClicked(self):
        # get index of target to delete
        index = self.ui.targets.currentIndex()
        print("index= ",index)
        # delete target from optimizer target listt
        self.optimizer.listOfPositions.pop(index)
        # np.sort(self.optimizer.listOfPositions)
        # update ui
        self.ui.targets.clear()
        self.ui.targets.addItems(["target_"+str(x)for x in range(len(self.optimizer.listOfPositions)-1)])
              
    def onDelObsButtonClicked(self):
        # get index of obstacle to delete
        index = self.ui.obstacles.currentIndex()
        # delete obstacle from optimizer obstacle list
        self.optimizer.listOfObstacle.pop(index)
        # update ui
        self.ui.obstacles.clear()
        self.ui.obstacles.addItems( "obstacle_"+str(x) for x in range(len(self.optimizer.listOfObstacle)) ) 
        
    
    # def __inputToTarget__(self, input:str):
    #     # transform string to list and elements to int
    #         # input.replace(" ", "")
    #         inputList = split(' ', input) 
    #         inputList = [int(x) for x in inputList]
        
    #     # add to targets list
    #         print([inputList[0], np.array(inputList[1:])])
    #         self.optimizer.listOfPositions.append( (inputList[0], np.array(inputList[1:])) )
    #         self.optimizer.listOfPositions.sort()
    #         self.ui.targets.clear()
    #         self.ui.targets.addItems(["t:"+str(x[0])+" x:"+str(x[1]) for x in self.optimizer.listOfPositions])          

    def __setOptimizer__(self):
        try:
        
            """Setup Optimizer"""
            self.optimizer = Optimizer(self.model)

            self.optimizer.Nsim = int(self.ui.Nsim.toPlainText())
            
            self.optimizer.setHorizon(int(self.ui.Horizon.toPlainText()))

            """Set Objective"""
            self.optimizer.setObjective()
            
            """Set Obstacle Constraints"""
            self.optimizer.listOfObstacle = self.obstacles
            print(self.optimizer.listOfObstacle)
            self.ui.obstacles.clear()
            self.ui.obstacles.addItems(["obstacle_"+ str(x) for x in range(len(self.optimizer.listOfObstacle))])
            
            self.optimizer.umin = int(self.ui.umin.toPlainText())
            
            self.optimizer.umax = int(self.ui.umax.toPlainText())
            
            self.optimizer.xmin = int(self.ui.xmin.toPlainText())
            
            self.optimizer.xmax = int(self.ui.xmax.toPlainText())

            self.optimizer.setObstacleConstraints()
            self.optimizer.setBoundsConstraints()

            # Set Trajectory
            self.optimizer.listOfPositions = self.targets
            self.optimizer.setTrajectory()

            self.optimizer.setup()

            """Set up simulator"""
            self.simulator = Simulator(self.optimizer)
            print(split(" ", self.ui.xinit.toPlainText()))
            self.simulator.xinit = np.array([int(x) for x in split(" ", self.ui.xinit.toPlainText())])
            # self.__setPlotList__()
            
            # self.ui_message("Initialized simulator with default system.", "Info")
            
        except:
            self.ui_message("Error launching plot.", "Error")
            e = str(sys.exc_info()[0]) + ": " + str(sys.exc_info()[1])
            raise Exception(e)
    
    def __setPlotList__(self):
        plots = self.simulator.getSimStruct(path=self.simStruct_path)[0]
        self.ui.plots.clear()
        self.ui.plots.addItems(plots)
    
    def __plotEnv__(self, ax):
        for i in range(len(self.optimizer.listOfObstacle)):
            polygon = Polygon(self.optimizer.listOfObstacle[i])
            ax.add_patch(polygon)
            ax.annotate(str(i), xy=centroid(self.optimizer.listOfObstacle[i]), ha='center', va='center')
        
        data = self.simulator.getSimStruct()[1]["Position"]
        targetIndex =  data["variables"].index("targetsim")
        print("target index= ", targetIndex)
        x, y = data["indexes"][targetIndex]
        
        for i in range(len(self.optimizer.listOfPositions)):
            ax.plot(self.optimizer.listOfPositions[i][x],self.optimizer.listOfPositions[i][y], 'r+')
            xpoint,ypoint = [self.optimizer.listOfPositions[i][x], self.optimizer.listOfPositions[i][y]]
            ax.annotate(str(i), xy=(xpoint,ypoint+0.1), ha='center', va='bottom')
        
app = QApplication(sys.argv)

window = MainWindow()
window.show()

app.exec()