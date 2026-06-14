from droneMQTT import droneInstance
import time
import threading
from SymbolicName import *
                 
if __name__ == '__main__':
    # Initialize a drone instance for system 
    drone1 = droneInstance("1")

    # Helping just enter thread once times
    lock = threading.Lock()
    while True:
        
        #Check master is online ? and send init just once times
        if drone1.masterSts == MASTER_ONLINE and drone1.sendInit == NOT_SEND_INIT:
            drone1.sendInit = SEND_INIT_SUCCESS

            #Drone instance send init message to master and other drone to sync drone number in system  
            drone1.clientInit.droneInit(DRONE_COM)

            # Print logger low layer of MQTT protocol 
            drone1.clientInit.logger()

            # This thread to enter the loop to get data from drone after send all info to CtrlSD panel (Mater)
            sendSysReportThread = threading.Thread(target=drone1.clientInit.sendFeedbackInfo, args=(lock,1,'udp:172.30.144.1:14550',)) 
            sendSysReportThread.start()

        # Handle payload receive from topic DRONE_COM with more different type
        drone1.handleData(drone1.clientRecvMsg)
        

 