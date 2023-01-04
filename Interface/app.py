
# from MPC.model import Model
from MPC.model import Model
from MPC.optimizer import Optimizer
from MPC.simulator import Simulator
from ui_MainWindow import Ui_MainWindow
import sys
from PyQt6.QtCore import QSize, Qt, QFileInfo
from PyQt6.QtWidgets import QApplication, QMainWindow, QMessageBox, QFileDialog, QPlainTextEdit
from PyQt6 import QtGui
import os
import numpy as np
import polytope as poly


# Subclass QMainWindow to customize your application's main window
class MainWindow(QMainWindow):
    
    # MPC objects
    model = Model()
    optimizer = None
    simulator = None
    
    obstacles = {}
    targets = {}
    
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
        
        # Set default system
        self.setDefaultSystem()
           

    def ui_message(self, msg:str, type:str="info"):
        dlg = QMessageBox(self)
        dlg.setWindowTitle(type)
        dlg.setText(msg)
        button = dlg.exec()

        if button == QMessageBox.StandardButton.Ok:
            dlg.close()
    
    def onLaunchButtonClicked(self):
        simStruct_path = os.path.join(os.getcwd(),"Interface", "ressources", "sim_struct.json")
        # simulate
        self.simulator.launchSimulation()
        self.simulator.show(self.ui.plots.currentText(), simStruct_path)   
    
    def __convertToList__(self, widget:QPlainTextEdit):
        pass
        
    def onSetSysButton(self):
        try:
            dlg = QFileDialog()
            # dlg.setLayout(QVBoxLayout()) #align content vertically
            fileName = dlg.getOpenFileName(self, 'Choose system file', '', 'System Files (*.json)')
            
            if fileName[0]:
                print(fileName[0])
                self.model.setAgentSystem(filepath=fileName[0])
                self.ui.currentModel.setMarkdown(QFileInfo.fileInfo(fileName[0].fileName())) # Update current model ui 
                self.ui_message("Operation Succeeded")
            
        except:
            self.ui_message("Operation Failed", "Error")
            e = str(sys.exc_info()[0]) + ": " + str(sys.exc_info()[1])
            print(e)
    
    def setDefaultSystem(self):
        
        try:
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
            self.optimizer.listOfObstacle.append(obs)
            self.ui.obstacles.addItem("square")
            self.obstacles["square"] = 0 # use index for delete
            
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

            # Set Trajectory
            self.optimizer.listOfPositions = [(15,np.array([5, 5, 0, 0])), (30,np.array([-3, 0, 0, 0])), (50,np.array([4, 1, 0, 0])) ] 
            self.optimizer.setTrajectory()
            self.ui.targets.addItems(["t:"+str(x[0])+" x:"+str(x[1]) for x in self.optimizer.listOfPositions])

            self.optimizer.setup()

            """Set up simulator"""
            self.simulator = Simulator(self.optimizer)
            self.simulator.xinit = np.array([2, 2, 0, 0])
            self.ui.xinit.setPlainText(str(self.simulator.xinit))
            
            self.__setPlotList__()
            
            self.ui_message("Initialized simulator with default system.", "Info")
            
        except:
            self.ui_message("Couldn't initialize default model.", "Error")
            e = str(sys.exc_info()[0]) + ": " + str(sys.exc_info()[1])
            print(e)

    # def __setSystem_(self):
        try:
            """ Set number of agents"""
            self.model.nb_agents = 1
        
            default_sys = os.path.join(os.getcwd(),"Interface", "ressources", "system.json")
            print(default_sys)
            self.model.setAgentSystem(filepath=default_sys)
            self.ui.currentModel.setMarkdown("system.json") # Update current model ui
            A, B, C, D = self.model.getAgentSystem()
            print("A='{}'\nB='{}'\nC='{}'\nD='{}'\n".format(A, B, C, D) )
            
            """setup model"""
            self.model.setupModel()
        
            """Setup Optimizer"""
            self.optimizer = Optimizer(self.model)

            self.optimizer.setHorizon(self.ui.Horizon.toPlainText())

            """Set Objective"""
            self.optimizer.setObjective()
            
            """Set Obstacle Constraints"""
            self.optimizer.listOfObstacle.append()
            self.ui.obstacles.itemData()
            self.ui.obstacles.addItem()
            
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

            # Set Trajectory
            self.optimizer.listOfPositions = [(15,np.array([5, 5, 0, 0])), (30,np.array([-3, 0, 0, 0])), (50,np.array([4, 1, 0, 0])) ] 
            self.optimizer.setTrajectory()

            self.optimizer.setup()

            """Set up simulator"""
            self.simulator = Simulator(self.optimizer)
            self.simulator.xinit = np.array([2, 2, 0, 0])
            
            self.__setPlotList__()
            
            self.ui_message("Initialized simulator with default system.", "Info")
            
        except:
            self.ui_message("Couldn't initialize default model.", "Error")
            e = str(sys.exc_info()[0]) + ": " + str(sys.exc_info()[1])
            print(e)
    
    def __setPlotList__(self):
        plots = self.simulator.getSimStruct()[0]
        self.ui.plots.addItems(plots)
    
    # def __setTrajectoryList(self, _pos:list):
        self.ui.targets.addItems(["t:"+str(x[0])+" x:"+str(x[1]) for x in _pos])   

app = QApplication(sys.argv)

window = MainWindow()
window.show()

app.exec()