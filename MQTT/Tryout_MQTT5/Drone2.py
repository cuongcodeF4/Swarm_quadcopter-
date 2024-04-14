from droneMQTT import droneMQTT
from droneMQTT import droneInstance
from queue import Queue
import time
import threading
from SymbolicName import *
                 
if __name__ == '__main__':
    drone2 = droneInstance("drone2")
    while True:
        if drone2.masterSts == MASTER_ONLINE and drone2.sendInit == NOT_SEND_INIT:
            drone2.sendInit = SEND_INIT_SUCCESS
            drone2.clientInit.droneInit(DRONE_COM)
            drone2.clientInit.logger()
        drone2.handleData(drone2.clientRecvMsg)

 