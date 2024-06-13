from MQTTProtocol import MQTTProtocol
from droneMQTT import droneInstance
from queue import Queue
import time
import threading
from SymbolicName import *
                 
if __name__ == '__main__':
    drone3 = droneInstance("3")
    lock = threading.Lock()
    while True:
        if drone3.masterSts == MASTER_ONLINE and drone3.sendInit == NOT_SEND_INIT:
            drone3.sendInit = SEND_INIT_SUCCESS
            drone3.clientInit.droneInit(DRONE_COM)
            drone3.clientInit.logger()
            sendSysReportThread = threading.Thread(target=drone3.clientInit.sendFeedbackInfo, args=(lock,3,'udp:172.30.144.1:14570',)) 
            sendSysReportThread.start()
        drone3.handleData(drone3.clientRecvMsg)
        