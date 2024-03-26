from droneMQTT import droneMQTT
from enum import Enum
import time

WAIT_TO_CONNECT = 1
SYS_COM = "topicSystemCom"
DRONE_COM = "topicDroneCom"
CONTROL_COM = "topicControlCom"

TOPIC_LAST_WILL = "Drone/LastWill"

LstData = ["command1","command2","command3"]
payload = None

if __name__ == '__main__':

    # Initial the Master to send command 
    Master = droneMQTT(client_id="Master")
    Master.connectBroker()
    Master.logger()
    time.sleep(WAIT_TO_CONNECT)

    #Send all command
    for cmd in LstData:
        Master.Client.loop_start()
        Master.publishMsg(topic= DRONE_COM, payload=cmd)
        Master.Client.loop_stop()

    MasterCheck = droneMQTT(client_id="CheckConnect")
    MasterCheck.connectBroker()
    time.sleep(WAIT_TO_CONNECT)
    MasterCheck.subscribe(topic=TOPIC_LAST_WILL)
    MasterCheck.Client.loop_forever()



