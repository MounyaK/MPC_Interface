
# from MPC.model import Model
from MPC.model import Model
from ui_MainWindow import Ui_MainWindow
import sys
from PyQt6.QtCore import QSize, Qt
from PyQt6.QtWidgets import QApplication, QMainWindow, QPushButton, QMessageBox, QFileDialog
from PyQt6 import QtGui


# Subclass QMainWindow to customize your application's main window
class MainWindow(QMainWindow):
    
    # MPC objects
    model = Model()
    optimizer = None
    simulator = None
    
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
        

    def ui_message(self, msg:str, type:str="info"):
        dlg = QMessageBox(self)
        dlg.setWindowTitle(type)
        dlg.setText(msg)
        button = dlg.exec()

        if button == QMessageBox.StandardButton.Ok:
            dlg.close()
    
    def onLaunchButtonClicked(self):
        print(self.ui.nbAgents.toPlainText())
        
    def onSetSysButton(self):
        try:
            fileName = QFileDialog.getOpenFileName(self, 'OpenFile')
            if fileName[0]:
                print(fileName[0])
                self.model.setAgentSystem(filepath=fileName[0])
                self.ui_message("Operation Succeeded")
            
        except:
            self.ui_message("Operation Failed", "Error")
            e = str(sys.exc_info()[0]) + ": " + str(sys.exc_info()[1])
            print(e)
            

app = QApplication(sys.argv)

window = MainWindow()
window.show()

app.exec()