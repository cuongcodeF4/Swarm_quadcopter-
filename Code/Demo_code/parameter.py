#cần viết 1 format chung cho các biến cần điều chỉnh, cũng như là 1 thư viện 
#để khi cần thì chỉ vào lấy biến ra theo format đã có từ trước
#có thể để các dạng thông tin trao đổi giữa các drone trong này, drone chỉ cần vào và lấy ra cho tiện

####MQTT para
#Use for set up client, publisher, sub,...
#User can change the info of the MQTT_ID base on the hardware and favor
#exg: ID = MQTT_ID("192.168.1.1", 1, 1883, None, None, None)
# >> need to be excute before using the MQTT pub/sub func
#when in use: ID.broker or ID.topic[1]
class MQTT_ID:
    def __init__(self, broker_IP, Num, port, client_id, username, password):
         #don't change and stay the same
        #can be seen as define
        #you can find it by : Go to  "Command Prompt" and input "ipconfig" to get your ip address
        self.broker_IP = broker_IP

        #Change upon the info of the drone it self.
        self.Num = Num
        #Base on the info of the drone itself
        self.port = port

        #Drone self info, don't need to be true are specific
        self.client_id = client_id
        self.username = username
        self.password = password










