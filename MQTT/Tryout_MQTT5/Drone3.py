from droneMQTT import droneMQTT
from droneMQTT import droneInstance
from queue import Queue
import time
import threading
from SymbolicName import *
                 
if __name__ == '__main__':
    drone3 = droneInstance("3")
    while True:
        if drone3.masterSts == MASTER_ONLINE and drone3.sendInit == NOT_SEND_INIT:
            drone3.sendInit = SEND_INIT_SUCCESS
            drone3.clientInit.droneInit(DRONE_COM)
            drone3.clientInit.logger()
        drone3.handleData(drone3.clientRecvMsg)

 