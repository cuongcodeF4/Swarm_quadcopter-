import sys
import os
import subprocess
from pathlib import Path
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5 import QtCore, QtGui
from datetime import datetime
import time
from SymbolicName import *

        
try:
    #Generate file ui python from file.ui
    dirPath = os.path.abspath('uiSystemDroneNew.py')
    pathFileUi = os.path.dirname(dirPath)
    os.remove(pathFileUi+"/uiSystemDroneNew.py")
except:
    pass
subprocess.run("python -m PyQt5.uic.pyuic systemControlDrone.ui -o uiSystemDroneNew.py", shell=True)

from uiSystemDroneNew import Ui_MainWindow

# class MasterHandeler(QThread):
#     handleLasWil      = pyqtSignal(object)
#     updateConsoleLog  = pyqtSignal(str,str,str)
    
#     # sendCmd           = pyqtSignal()
#     # updateDroneStatus = pyqtSignal()
#     def __init__(self,func):
#         super().__init__()
#         self.function = func
#         print("[DEBUG] a new thread was spawned")
#     def run(self):
#         while True:
#             self.function()

class MasterInit(QThread):
    updateButton      = pyqtSignal()
    updateConsoleLog  = pyqtSignal(str,str)
    checkConnectDrone = pyqtSignal(int)
    def __init__(self,func,master):
        super().__init__()
        self.func = func
        self.masterPC = master
        print("[DEBUG] Threading init master")
    def run(self):
        self.func()
        while not self.masterPC.log.empty():
            log = self.masterPC.log.get()
            self.updateConsoleLog.emit("INFO",log,)
        while True:
            self.checkConnectDrone.emit(2)

            

class MyWindow(QMainWindow):
    dirname = os.path.dirname(__file__)
    def __init__(self):
        super().__init__()       

        # Initialize the UI class
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # set the title
        self.setWindowTitle("Swarm Drone")
        # set icon for application 
        pathIcon = os.path.join(self.dirname, 'Images/icon_drone.png')
        self.setWindowIcon(QtGui.QIcon(pathIcon))

        # Connect the signal 'textChanged' of QLineEdit to a slot
        self.ui.nrbOfDroneLineEdit.returnPressed.connect(self.updateDroneStatus)
        # Initial number of drones
        self.num_drones = 0

        # Initialize list to store drone labels
        self.drone_labels = []   

        self.commandList = ["Choose the command","Arm","Takeoff","Land"]       
        self.ui.commandComboBox.addItems(self.commandList)

        self.ShapeList = ["Choose the shape","Circle","Square","straight line"]       
        self.ui.shapeComboBox.addItems(self.ShapeList)

        self.typeList = ["All Drones","One Drone"]       
        self.ui.typeControlComboBox.addItems(self.typeList)
        self.ui.typeControlComboBox.activated.connect(self.enableSelectDrone)
        self.ui.startMaster.clicked.connect(self.runMaster)
        

    def runMaster(self):
        from Sender import Master 
        self.master = Master()
        if not self.ui.nrbOfDroneLineEdit.text():
            self.printLog("WARNING","Please enter the number of drones!")
            QMessageBox.warning(self, "Warning", "Please enter the number of drones!")          
            return
        else:      
            self.masterInit = MasterInit(self.master.masterConnectBroker,self.master)
            self.masterInit.start()
            self.masterInit.updateConsoleLog.connect(self.printLog)
            self.masterInit.updateButton.connect(self.updateBntStartMaster)
            self.masterInit.checkConnectDrone.connect(self.master.masterCheckConnect)
            time.sleep(4)
            self.masterInit.updateButton.emit()
            self.num_drones = int(self.ui.nrbOfDroneLineEdit.text())
            # while not self.master.stsConnectBroker == -1:
            #     self.masterInit.updateButton.emit()
            #     break

            
            # if  self.master.masterRecvLW.log == "Success":
            #     self.masterInit.updateConsoleLog.emit("INFO","Connected to MQTT Broker!","Master")
            
            
            # try:
            #     self.printLog("DEBUG","Out thread ")
            #     # Get the number of drones from the line edit
            #     self.num_drones = int(self.ui.nrbOfDroneLineEdit.text())
            # except:
            #     print("[ERORR] Incorrect type.Please input an integer number")
            #     return
            # self.master.masterCheckConnect(self.num_drones)

            # self.worker = MasterHandeler(self.master.MasterReceiveLW)
            # self.worker.handleLasWil.connect(self.master.handleLW)
            # self.worker.sendCmd.connect(self.master.masterSendCommand)
            # # self.worker.updateDroneStatus.connect(self.updateWaitTable)
            # self.worker.updateConsoleLog.connect(self.printLog)
            # self.worker.start()


    def enableSelectDrone(self):
        selected_type = self.ui.typeControlComboBox.currentText()
        if selected_type == "One Drone":
            self.ui.label_6.setEnabled(True)
            self.ui.droneNrmControlLineEdit.setEnabled(True)
        else:
            self.ui.label_6.setEnabled(False)
            self.ui.droneNrmControlLineEdit.setEnabled(False)
    def updateDroneStatus(self):
        checkSize    = False
        asymmetrical = False
        # Clear existing labels
        for label in self.drone_labels:
            label.deleteLater()
            self.drone_labels = []
        try:
            # Get the number of drones from the line edit
            self.num_drones = int(self.ui.nrbOfDroneLineEdit.text())
        except:
            print("[ERORR] Incorrect type.Please input an integer number")
        

        
        pathDroneImage = os.path.join(self.dirname, 'Images/drone_black.png')
        self.droneImage = QtGui.QPixmap(pathDroneImage)
        size = 100
        # Get geometry of droneStatusGroupBox
        grpBoxDroneSts = self.ui.droneStatusGroupBox.geometry()
        widthGrpBox = grpBoxDroneSts.width()
        heightGrpBox = grpBoxDroneSts.height()

        rColCouple = []
        posList = []
        for i in range(1,self.num_drones+ 1):
            integer =   self.num_drones // i
            remainder =  self.num_drones % i
            if integer < i:
                break
            rColCouple.append([i,integer,integer+ remainder])

        # Check size of row and column . Do have fit with group box
        rowAddDrone  = rColCouple[-1][-1] -  rColCouple[-1][-2]
        print("[DEBUG] rowAddDrone= ",rowAddDrone)
        if rowAddDrone != 0 :
            maxRow = rColCouple[-1][-3]
            maxCol = rColCouple[-1][-2] + 1
            while checkSize == False:
                if ((maxCol * (size+ size)) > widthGrpBox) or ((maxRow * (size+ size//2)) > heightGrpBox):
                    size = int(size - size *0.2)
                else:
                    checkSize = True
            paddingWidth = (widthGrpBox -(maxCol * (size+ size))) //2
            paddingHeight = (heightGrpBox - (maxRow * (size+ size//2))) //2 

            posX_Width = paddingWidth + size/2
            posY_Height = paddingHeight + size/4

            paddingWidth0 = (widthGrpBox -((maxCol-1) * (size+ size))) //2
            paddingHeight0 = (heightGrpBox - (maxRow * (size+ size//2))) //2 
            posX_Width0 = paddingWidth0 + size/2
            posY_Height0 = paddingHeight0 + size/4

            for row in range(maxRow):
                if row >= (maxRow - rowAddDrone):
                    actualCol = maxCol
                    posX = posX_Width
                    posY = posY_Height
                else:
                    actualCol = maxCol - 1
                    posX = posX_Width0
                    posY = posY_Height0

                for col in range(actualCol):
                    x = posX + col* int( size + size)
                    y = posY + row* int( size + size/2)
                    posList.append([x,y])
        else:
            maxRow = rColCouple[-1][-3]
            maxCol = rColCouple[-1][-2] 
            while checkSize == False:
                if ((maxCol * (size+ size)) > widthGrpBox) or ((maxRow * (size+ size//2)) > heightGrpBox):
                    size = size - size *0.2
                else:
                    checkSize = True
                paddingWidth = (widthGrpBox -(maxCol * (size+ size))) /2
                paddingHeight = (heightGrpBox - (maxRow * (size+ size//2))) /2 
                posX_Width = paddingWidth + size/2
                posY_Height = paddingHeight + size/4
            for row in range(maxRow):
                for col in range(maxCol):
                    x = posX_Width + col* ( size + size)
                    y = posY_Height + row* ( size + size/2)
                    posList.append([x,y])
        
        self.scaled_pixmap = self.droneImage.scaled(size, size)
        print("[DEBUG] Size= ",size)
        print("[DEBUG] heightGrpBox= ",heightGrpBox)
        print("[DEBUG] posList= ",posList)

        for nbrDrone in range(self.num_drones):
            label = QLabel(self.ui.droneStatusGroupBox)
            label.setGeometry(posList[nbrDrone][0], posList[nbrDrone][1], size, size)
            label.setPixmap(self.scaled_pixmap)
            label.setVisible(True)  # Make label visible
            self.drone_labels.append(label)
    def printLog(self,type,log):
        timeLog = datetime.now()
        time = str(timeLog.hour).zfill(2)+":"+str(timeLog.minute).zfill(
                    2)+":"+str(timeLog.second).zfill(2)  
        
        self.ui.statusLog.append("[Timestamp:{}] [{}] {}".format(time,type,log))
    def updateBntStartMaster(self):
        if self.master.stsConnectBroker == CONNECT_SUCCESS:
            self.ui.startMaster.setStyleSheet("color: green;")
        elif self.master.stsConnectBroker == CONNECT_FAILED:
            self.ui.startMaster.setStyleSheet("color: red;")
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MyWindow()
    window.show()
    sys.exit(app.exec_())