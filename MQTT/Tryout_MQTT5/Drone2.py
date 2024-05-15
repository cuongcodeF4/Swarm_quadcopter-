from droneMQTT import droneInstance
import time
import threading
from SymbolicName import *
                 
if __name__ == '__main__':
    # Initialize a drone instance for system 
    drone2 = droneInstance("2")

    # Helping just enter thread once times
    lock = threading.Lock()
    while True:
        
        #Check master is online ? and send init just once times
        if drone2.masterSts == MASTER_ONLINE and drone2.sendInit == NOT_SEND_INIT:
            drone2.sendInit = SEND_INIT_SUCCESS

            #Drone instance send init message to master and other drone to sync drone number in system  
            drone2.clientInit.droneInit(DRONE_COM)

            # Print logger low layer of MQTT protocol 
            drone2.clientInit.logger()

            # This thread to enter the loop to get data from drone after send all info to CtrlSD panel (Mater)
            sendSysReportThread = threading.Thread(target=drone2.clientInit.sendFeedbackInfo, args=(lock,2,'udp:172.30.144.1:14560',)) 
            sendSysReportThread.start()

        # Handle payload receive from topic DRONE_COM with more different type
        drone2.handleData(drone2.clientRecvMsg)
        

 