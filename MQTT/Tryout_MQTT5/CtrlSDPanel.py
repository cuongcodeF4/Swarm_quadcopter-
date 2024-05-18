import sys
import os
import subprocess
from pathlib import Path
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5 import QtCore, QtGui
from PyQt5.QtGui import QFont
from datetime import datetime
import time
from SymbolicName import *
from queue import Queue
import ujson
        
try: 
    #Generate file ui python from file.ui
    dirPath = os.path.abspath('uiSystemDrone.py')
    pathFileUi = os.path.dirname(dirPath)
    os.remove(pathFileUi+"/uiSystemDrone.py")
except:
    pass
subprocess.run("python -m PyQt5.uic.pyuic systemControlDrone.ui -o uiSystemDrone.py", shell=True)

from uiSystemDrone import Ui_MainWindow

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
    checkConnectDrone = pyqtSignal()
    updateDroneSts    = pyqtSignal()
    updateBatDrone    = pyqtSignal()
    updateCmdPending  = pyqtSignal()
    def __init__(self,func,master,dataSend):
        super().__init__()
        self.func = func
        self.masterPC = master
        self.log = Queue()
        self.dataSend = dataSend
        print("[DEBUG] Threading init master")
    def run(self):
        self.func()
        self.log.put(self.masterPC.masterRecvLW.log.get())
        self.log.put(self.masterPC.Master.log.get())
        self.updateButton.emit()
        while True:
            while not (self.log.empty()):
                log = self.log.get()
                self.updateConsoleLog.emit("INFO",log)
            # Check the number of drone was connected with broker 
            self.checkConnectDrone.emit()
            if not self.masterPC.logMaster.empty():
                self.log.put(self.masterPC.logMaster.get())  
            if self.masterPC.connectStatus != None or self.masterPC.FcConnectStatus == True:
                print("[DEBUG] update drone status")
                self.updateDroneSts.emit()
                self.masterPC.connectStatus = None # Help the signal only callback when client connect or disconnect with broker 
                self.masterPC.FcConnectStatus = False    
            if self.masterPC.updateStsBat == ON :
                self.masterPC.updateStsBat = OFF
                self.updateBatDrone.emit()
            if self.masterPC.missionDone == True:
                self.masterPC.missionDone = False
                self.updateCmdPending.emit()

            time.sleep(0.1) 

class MyWindow(QMainWindow):
    dirname = os.path.dirname(__file__)
    def __init__(self):
        super().__init__()       

        # Initialize the UI class
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        
        # set the title
        self.setWindowTitle("Swarm Drone Control Panel")
        
        # set image for side panel 
        pathIcon = os.path.join(self.dirname, 'Images/drone_swarm.webp')
        self.ui.label_17.setPixmap(QtGui.QPixmap(pathIcon))

        # set icon for application 
        pathIcon = os.path.join(self.dirname, 'Images/icon_drone.png')
        self.setWindowIcon(QtGui.QIcon(pathIcon)) 
        

        # Connect the signal 'textChanged' of QLineEdit to a slot
        self.ui.nrbOfDroneLineEdit.returnPressed.connect(self.updateDroneStatus)
        # Initial number of drones
        self.num_drones = 0

        # Initialize list to store drone labels
        self.drone_labels = []  

        # List to store battery of drone corresponding ID
        self.batteryDrone = []  

        #Queue contain data to send to drone
        self.dataSend = Queue()
        #payload to send 
        self.payload = {}

        # Include All and Unit select command type
        self.typeCmd = ALL

        # Initial default size of image drone was created 
        self.sizeImage = 100

        self.commandList = ["Choose the command","Arm","Disarm","Takeoff","Land","Prepare act"]       
        self.ui.commandComboBox.addItems(self.commandList)

        self.ShapeList = ["Choose the shape","Circle","Square","straight line"]       
        self.ui.shapeComboBox.addItems(self.ShapeList)

        self.typeList = ["All Drones","One Drone"]       
        self.ui.typeControlComboBox.addItems(self.typeList)
        self.ui.typeControlComboBox.activated.connect(self.enableSelectDrone)
        self.ui.shapeComboBox.activated.connect(self.enableCommandMission)
        self.ui.startMaster.clicked.connect(self.runMaster)
        self.ui.stopMaster.clicked.connect(self.stopMaster)
        self.ui.bntSendCommand.clicked.connect(self.sendCommand)
        # Variable to check if master online is do something  
        self.run = 0 
        # Variable to get name command to display on command pending area after send command
        self.currentCmd = ""
        
    #This function to start connect master to broker  
    def runMaster(self):
        # Check drone number was input ? 
        if not self.ui.nrbOfDroneLineEdit.text():
            self.printLog("WARNING","Please enter the number of drones!")
            QMessageBox.warning(self, "Warning", "Please enter the number of drones!")          
            return
        else:
            if self.run == 0:
                self.run = 1
                from Sender import Master 
                self.num_drones = int(self.ui.nrbOfDroneLineEdit.text())
                # Initial class Master from Sender.py file 
                self.master = Master(self.num_drones) 

                # Creating Qthread to handle event when init master              
                self.masterInit = MasterInit(self.master.masterConnectBroker,self.master,self.dataSend)

                # Signal of Qthread Master init 
                self.masterInit.start()
                self.masterInit.updateConsoleLog.connect(self.printLog)
                self.masterInit.updateButton.connect(self.updateBntStartMaster)
                self.masterInit.checkConnectDrone.connect(self.master.masterCheckConnect)
                self.masterInit.updateDroneSts.connect(self.showDroneConnect)
                self.masterInit.updateBatDrone.connect(self.updateBattery)
                self.masterInit.updateCmdPending.connect(self.removeCmdPending)

    # This function to stop master 
    def stopMaster(self):
        self.printLog("INFO","Master was disconnected with broker")
        self.ui.startMaster.setStyleSheet("color: black;")
        self.master.masterStopConnect()
        self.updateDroneStatus()
        self.masterInit.quit()
        self.run = 0

    def enableSelectDrone(self):
        selected_type = self.ui.typeControlComboBox.currentText()
        if selected_type == "One Drone":
            self.typeCmd = UNIT
            self.ui.Drone.setEnabled(True)
            self.ui.droneNrmControlLineEdit.setEnabled(True)
        else:
            self.typeCmd = ALL 
            self.ui.Drone.setEnabled(False)
            self.ui.droneNrmControlLineEdit.setEnabled(False)
        
    def enableCommandMission(self):
        shapeCmdEnable = self.ui.shapeComboBox.currentText()
        if shapeCmdEnable == "Choose the shape":
            self.ui.command.setEnabled(True)
            self.ui.commandComboBox.setEnabled(True)
        else:
            self.ui.command.setEnabled(False)
            self.ui.commandComboBox.setEnabled(False)

            
    # Format payload when send to all drone
    def DATA_ALL_TYPE(self,CMD, PARA=10, ALT=None, LON=None, LAT=None):
            return {
                "TYPE" : ALL,
                "ALL_CMD" : {
                    "CMD" : CMD,
                    "PARA": PARA,
                    "ALT" : ALT,
                    "LON" : LON,
                    "LAT" : LAT}
            }
    
    # Format payload when send to any drone 
    def DATA_UNIT_TYPE(self,UNIT_SELECTED, CMD, PARA=10, ALT=None, LON=None, LAT=None):    
            return {
                "TYPE" : UNIT,
                "UNIT_CMD" : {
                    "UNIT_SELECTED" : UNIT_SELECTED,
                        "CMD" : CMD,
                        "PARA": PARA,
                        "ALT" : ALT,
                        "LON" : LON,
                        "LAT" : LAT
                        }
                    }
    
    # Add command is selected after push Send command button
    def addCommand(self):
        self.payload= {}
        # Send basic command ( Arm, takeoff, land,...) 
        if self.ui.shapeComboBox.currentText() == "Choose the shape" : 
            cmd = self.ui.commandComboBox.currentText() 
            if cmd != "Choose the command":
                self.currentCmd = cmd
                alt = (self.ui.altValue.text())
                long = (self.ui.longValue.text())
                lat = (self.ui.latValue.text())
                para = None
                if self.typeCmd == ALL:
                    self.payload = self.DATA_ALL_TYPE(cmd,para,alt,long,lat)
                elif self.typeCmd == UNIT:
                    droneSelected = self.ui.droneNrmControlLineEdit.text()
                    self.payload = self.DATA_UNIT_TYPE(droneSelected,cmd,para,alt,long,lat)

        # Send a shape to drones follow 
        else:
            cmd   =  self.ui.shapeComboBox.currentText()
            self.currentCmd = cmd
            para  =  self.ui.paraValue.text()
            alt   =  self.ui.altValue.text()
            long  =  self.ui.longValue.text()
            lat   =  self.ui.latValue.text()
            self.payload = self.DATA_ALL_TYPE(cmd,para,alt,long,lat)

    def sendCommand(self):
        self.addCommand()
        # Check master online?
        if self.run ==1:
            if self.currentCmd == "Circle" or self.currentCmd == "Takeoff":
                if  self.ui.altValue.text() != "":
                    #Checking the drone number are enough?
                    if self.master.droneConnected == self.num_drones:
                        # Add command to command pending table 
                        self.ui.commandPending.append(self.currentCmd)
                        # Convert payload to ujon format
                        self.dataSend.put(ujson.dumps(self.payload))
                    # Call function masterSendCommand from Sender.py to send payload to drones
                    self.master.masterSendCommand(self.dataSend)
                    print("Queue size after clearing:", self.dataSend.qsize())
                else:
                    self.printLog("WARNING","Please enter altitude value!")
                    QMessageBox.warning(self, "Warning", "Please enter altitude value!")        
            else:
                #Checking the drone number are enough?
                    if self.master.droneConnected == self.num_drones:
                        # Add command to command pending table 
                        self.ui.commandPending.append(self.currentCmd)
                        # Convert payload to ujon format
                        self.dataSend.put(ujson.dumps(self.payload))
                    # Call function masterSendCommand from Sender.py to send payload to drones
                    self.master.masterSendCommand(self.dataSend)
                    print("Queue size after clearing:", self.dataSend.qsize())  
            return

    def removeCmdPending(self):
        text = self.ui.commandPending.toPlainText()  # Get the entire text from the QTextEdit
        lines = text.split('\n')  # Split the text into lines
        if lines:  # Ensure there is at least one line
            lines.pop(0)  # Remove the first line
        new_text = '\n'.join(lines)  # Join the remaining lines
        self.ui.commandPending.setPlainText(new_text)  # Set the modified text back to the QTextEdit


    def updateDroneStatus(self):
        self.ui.commandComboBox.setCurrentIndex(0)
        checkSize    = False
        self.payload= {}
        # asymmetrical = False

        # Clear existing images drone
        for label in self.drone_labels:
            label.deleteLater()
            self.drone_labels = []

        for bat in self.batteryDrone:
            bat.deleteLater()
            self.batteryDrone = []
        try:
            # Get the number of drones from the line edit
            self.num_drones = int(self.ui.nrbOfDroneLineEdit.text())
            if self.num_drones <= 0:
                return
        except:
            print("[ERORR] Incorrect type.Please input an integer number")
        

        #Get the path of image drone
        pathDroneImage = os.path.join(self.dirname, 'Images/drone_black.png')
        self.droneImageOff = QtGui.QPixmap(pathDroneImage)

        #Origin size of image
        size = 100

        # Get geometry of droneStatusGroupBox
        grpBoxDroneSts = self.ui.droneStatusGroupBox.geometry()
        widthGrpBox = grpBoxDroneSts.width()
        heightGrpBox = grpBoxDroneSts.height()

        rColCouple = []

        #List contain the position of drone images in group box
        self.posList = []

        #Get the row , col and the remainder of drone compare with col 
        for i in range(1,self.num_drones+ 1):
            integer =   self.num_drones // i
            remainder =  self.num_drones % i
            if integer < i:
                break
            rColCouple.append([i,integer,integer+ remainder])

        # Check size of row and column . Do have fit with group box
        rowAddDrone  = rColCouple[-1][-1] -  rColCouple[-1][-2]
        if rowAddDrone != 0 :
            maxRow = rColCouple[-1][-3]
            maxCol = rColCouple[-1][-2] + 1

            while checkSize == False:
            #if sum of width and height of all images bigger of the width and height of group box will reduce the size of images 20%
                if ((maxCol * (size+ size)) > widthGrpBox) or ((maxRow * (size+ size//2)) > heightGrpBox):
                    size = int(size - size *0.2)
                    self.sizeImage = size
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

            #Calculate the position of image in the situation asymmetrical
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
                    self.posList.append([x,y])
        else:
            maxRow = rColCouple[-1][-3]
            maxCol = rColCouple[-1][-2] 
            while checkSize == False:
                if ((maxCol * (size+ size)) > widthGrpBox) or ((maxRow * (size+ size//2)) > heightGrpBox):
                    size = size - size *0.2
                    self.sizeImage = size
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
                    self.posList.append([x,y])
        
        self.scaled_pixmap = self.droneImageOff.scaled(size, size)

        for nbrDrone in range(self.num_drones):
            #Create drone instances are pictures 
            label = QLabel(self.ui.droneStatusGroupBox)
            label.setGeometry(self.posList[nbrDrone][0], self.posList[nbrDrone][1], size, size)
            
            #Show battery of each drone
            self.progressBar = QProgressBar(self.ui.droneStatusGroupBox)
            self.progressBar.setGeometry(self.posList[nbrDrone][0], self.posList[nbrDrone][1] +size+size/15, size, size/10)  # Set position and size using x, y coordinates
            # self.progressBar.setValue(100)
            self.progressBar.setVisible(True)       
            self.progressBar.setTextVisible(False)
            label.setPixmap(self.scaled_pixmap)
            label.setVisible(True)  # Make label visible
            self.drone_labels.append(label)
            self.batteryDrone.append(self.progressBar)
            
    def updateBattery(self):
        self.listBat = self.master.listBattery
        for index in range(len(self.listBat)):
            progress = self.batteryDrone[index]
            if self.listBat[index] <= 20:
                progress.setStyleSheet("""
                QProgressBar::chunk {
                    background-color: red; /* Set the color of the filled portion */
                }                        
                """)
            else:
                progress.setStyleSheet("")
            progress.setValue(self.listBat[index])
            
    def resizeEvent(self, event):
        # Call your function here
        if self.ui.nrbOfDroneLineEdit.text(): 
            self.updateDroneStatus()
            if self.run == 1:
                self.showDroneConnect()
            event.accept()
    def showDroneConnect(self):
        for label in self.drone_labels:
            label.deleteLater()
            self.drone_labels = []
        for nbrDrone in range(self.num_drones):
            print("[DEBUG]Drone off:",nbrDrone)
            label = QLabel(self.ui.droneStatusGroupBox)
            label.setGeometry(self.posList[nbrDrone][0], self.posList[nbrDrone][1], self.sizeImage, self.sizeImage)
            label.setPixmap(self.scaled_pixmap)
            label.setVisible(True)  # Make label visible
            self.drone_labels.append(label)

        for droneOn in (self.master.droneConnectList):
            print("[DEBUG]Drone on:",droneOn)
            # self.drone_labels[droneOn-1].deleteLater()
            pathDroneImage = os.path.join(self.dirname, 'Images/drone_green.png')
            self.droneImageOn = QtGui.QPixmap(pathDroneImage)
            scaled_pixmapOn = self.droneImageOn.scaled(self.sizeImage, self.sizeImage)
            label = QLabel(self.ui.droneStatusGroupBox)
            label.setGeometry(self.posList[droneOn-1][0], self.posList[droneOn-1][1], self.sizeImage, self.sizeImage)
            label.setPixmap(scaled_pixmapOn)
            label.setVisible(True)
            self.drone_labels.append(label)
        for FcDisconnect in (self.master.pixhawkConnectList):
            print("[DEBUG]FC off:",FcDisconnect)
            # self.drone_labels[droneOn-1].deleteLater()
            pathDroneImage = os.path.join(self.dirname, 'Images/drone_red.png')
            self.droneImageOn = QtGui.QPixmap(pathDroneImage)
            scaled_pixmapOn = self.droneImageOn.scaled(self.sizeImage, self.sizeImage)
            label = QLabel(self.ui.droneStatusGroupBox)
            label.setGeometry(self.posList[FcDisconnect-1][0], self.posList[FcDisconnect-1][1], self.sizeImage, self.sizeImage)
            label.setPixmap(scaled_pixmapOn)
            label.setVisible(True)
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
            # self.master.masterStopConnect()
            self.masterInit.quit()
            self.masterInit.wait()
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MyWindow()
    window.show()
    sys.exit(app.exec_())