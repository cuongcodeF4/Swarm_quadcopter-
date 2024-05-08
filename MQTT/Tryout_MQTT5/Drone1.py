from droneMQTT import droneMQTT
from droneMQTT import droneInstance
from queue import Queue
import time
import threading
from SymbolicName import *
                 
if __name__ == '__main__':
    drone1 = droneInstance("1")
    lock = threading.Lock()
    while True:
        if drone1.masterSts == MASTER_ONLINE and drone1.sendInit == NOT_SEND_INIT:
            drone1.sendInit = SEND_INIT_SUCCESS
            drone1.clientInit.droneInit(DRONE_COM)
            drone1.clientInit.logger()
            sendSysReportThread = threading.Thread(target=drone1.clientInit.sendFeedbackInfo, args=(lock,1,'udp:172.30.144.1:14550',)) 
            sendSysReportThread.start()
        drone1.handleData(drone1.clientRecvMsg)
        

 