#this lib is use only for the drones in the swarm 
#you can add mor lib and function to this lib in order to make it work better with drone 
#Both of the lib is pre build with pub and sub func where it can automatic check for whether the connection
#is successfull or not. More over, both of the pub and sub func is the same for both and was simplify down so that
#all you need to do is give it a dict and it will pub for you.

#A drone only pub to topic "droneFB" and take all the info back to save and analyst
# but on the other side, drone need to sub info into 3 main topic sysCom, droneCom, conCom



#para usage: ID, msg (input form)
    #ID = [broker_IP, Num, port, topic, client_id, username, password]
        #broker >> const
        #port >>c const
        #topic >> ["droneFB", "sysCom", "droneCom", "conCom"]
    #msg = {[info1],[info2],[info3]}


import json
import time
import paho.mqtt.client as mqtt_client                          #need to be install using pip install paho-mqtt if not have



def on_connect(client, userdata, flags, rc):                    #Connection call back, use to check wether the connection is good or not
    #check ing call back value
    if rc == 0:
        #call back connected, push out msg and print successfully
        print("Connected to MQTT Broker!")
    else:
        #opposite value to connect successfully
        print("Failed to connect, return code %d\n", rc)



#before init this func please create an object with para lib by set ID = MQTT_ID()
#please also set all the para to the right value so that you can use it right
#the input ID will be in class form with preset value that we set in the setup phase
def publish(ID, msg):                                          
    #create client info with the input ID
    client = mqtt_client.Client()

    #check call back as a fail safe
    client.on_connect = on_connect
    #Trying to publish until fail (5 time)

    #set up client with the ID info
    client.connect(ID.broker_IP, ID.port)
    # Wait for connection to complete
    client.loop_start()
    # Publish the empty dictionary (converted to JSON)
    result = client.publish(ID.topic, json.dumps(msg))
    #safe result for call back function
    #result format is 0,1 and 2 with 0 indicate for successful publish
    #call back when success publish msg
    if result[0]==0:
        print(f"Send `{msg}` to topic:`{ID[3]}`")
        #Stop loop after connect and publish successfully
        client.loop_stop()
        #Disconnect MQTT server
        client.disconnect()
    else:
         print(f"Publish failed to topic: `{ID.topic}`, trying to re-publish")




def on_message(client, userdata, msg):                                  #Message receive call back
        msg_recv = msg.payload.decode()
        msg_recv_dict = json.loads(msg_recv)
        msgReceive = msg_recv_dict["drone1"]
        return msgReceive


def subscribe(ID, msg):
    #create client info with the input ID
    client = mqtt_client.Client(ID.client_id)
    client.username_pw_set(ID.username, ID.password)

    #check call back as a fail safe
    client.on_connect = on_connect

    #Trying to sub until fail (5 times)
    #set up client with the ID info
    client.connect(ID.broker, ID.port)
    # Wait for connection to complete
    client.loop_start()
    #Sub to the broker and try to received msg
    result = client.subscribe(ID.topic)
    client.on_message = on_message
    msgReceived = client.on_message
    if result[0] ==0:
        print(f"Received `{msgReceived}` from `{ID.topic}` topic")
        #Stop loop after connect and sub successfully
        client.loop_stop()
        #Disconnect MQTT server
        client.disconnect()
        return msgReceived
    else:
         print(f"Receive fail, trying to connect to `{ID.topic}` again")




###### MQTT commute
class pub:
    commandType = ''   #sysCom or droneCom or conCom, this will be the topic that the drone sub to
    pub_msg = {}       # An empty dict to store all the data and the info that gonna be pb to the broker

    #scan all the topic avaiable until msg is send or end
    if commandType == 'sysCom':
        #Bao gồm các gói lệnh và câu lệnh cho iệc kiểm tra và test các drone
        #Bao gồm luôn setup chế độ chuyển chế độ, fail safe check
        #exg: stop, hold, report, init, comCheck, setPara.
        None
    elif commandType == 'droneCom':
        #Tuỳ chỉnh drone, chọn drone để điều khiển, chọn chế độ bay chế độ điều khiển drone
        #assign vị trí và trả tín hiệu về máy chủ tức thời
        None
    else:
        #chưa nghĩ ra tác dụng cho lắm nên để đấy dùng sau 
        None

class msg:
    pub_msg = {}
    #maybe something like this 
    #{
    #    ['info1'],
    #    ['info2'],
    #    ['info3']
    #}


    sub_msg = ''

#cần add them para msg, có các trường hợp sẽ pub ra cho router, làm. một hàm duyệt tâts cả các biến
# được thay đổi và chỉ cho những biến đó được hiển thị trong msg khi pub
# forma khi chỉnh có thể là parameter.msg.<inf_id> = <value>