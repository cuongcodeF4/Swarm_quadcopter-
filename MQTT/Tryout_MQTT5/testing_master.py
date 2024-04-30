import masterMQTT
from SymbolicName import *
import time

master = masterMQTT.masterMQTT(client_id="master")
# Initial the Master to send command 
master.connectBroker(typeClient= TYPE_CLIENT_MASTER)
master.logger()
time.sleep(WAIT_TO_CONNECT)

