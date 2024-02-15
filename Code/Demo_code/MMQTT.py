#This lib is use ony for the master laptop
#you can add more func to this lib in order to make the MQTT protocol more fast and effective
#Both of the lib is pre build with pub and sub func where it can automatic check for whether the connection
#is successfull or not. More over, both of the pub and sub func is the same for both and was simplify down so that
#all you need to do is give it a dict and it will pub for you.

#A master only sub to topic "droneFB" and take all the info back to save and analyst
# but on the other side, master need to pub info into 3 main topic sysCom, droneCom, conCom




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
def on_message(client, userdata, msg):                                  #Message receive call back
        msg_recv = msg.payload.decode()
        msg_recv_dict = json.loads(msg_recv)
        msgReceive = msg_recv_dict["drone1"]
        return msgReceive

def getPub_msg():
    #return format ["topic", msg]
    #chose topic to pub to 
    available_topic = ['sysCom', 'droneCom', 'conCom', 'back']
    while True:
        topic = input('Topic select [sysCom][droneCom][conCom][Back]: ')
        #scan to see if the user input the right command
        if topic not in available_topic:
            print('ERROR COMMAND')
            topic = input('Topic select [sysCom][droneCom][conCom][Back]: ')
        #after it have taken the input value it will continue to taek in command
        if topic == "sysCom":
            available_cmd = ['comCheck', 'posReport', 'sysReport', 'arm', 'takeoff','land','initiate', 'back']
            #chose the fitted command
            cmd = input('Control Command: ')
            #scan to see if the user input the right command
            if cmd not in available_cmd:
                print("ERROR COMMAND")
                cmd = input('Control Command: ')
            #After finish choosing the right command
            #let it return the topic choosing with a back option if being choose
            if cmd == 'back':
                #let the user choose the topic once again
                #no need to run the whole thing again because we can eliminate this topic out
                topic = input('Topic select [droneCom][conCom][Back]: ')
                #scan to see if the user input the right command
                if topic not in available_topic:
                    print('ERROR COMMAND')
                    topic = input('Topic select [droneCom][conCom][Back]: ')
            else:
                value = input('Input value: ')
                #scan for insufficient value
                if value == None:
                    print("ERROR VALUE")
                    value = input('Input value: ')
                msg = [cmd,value]
                return [topic, msg]                                     #return format ["topic",["command","value"]]
        elif topic == "droneCom":
            available_cmd = ['unitTest','groupTest','']
            cmd = input('Control Command: ')
            if cmd not in available_cmd:
                print("ERROR COMMAND")
                cmd = input('Control Command: ')
            value = input('Unit or Group NUMs: ')
            msg = [cmd,value]
            return [topic,msg]
        elif topic == "conCom":
            available_cmd = ''
            pass
            return None
        elif topic == "back":
            break
        else:
            print("ERROR COMMAND")
            return None

def publish(ID):                                       #Publish function, msg is in dict format
    #para usage: ID, msg
    #ID = [broker, port,...] có nên để topic vào không?


    #take in message
    msg = getPub_msg()                                     #msg = ["topic", msg] or ["topic",["command","value"]]

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
    result = client.publish(msg[0], json.dumps(msg[1]))
    #safe result for call back function
    #result format is 0,1 and 2 with 0 indicate for successful publish
    #call back when success publish msg
    if result[0]==0:
        print(f"Send `{msg[1]}` to topic:`{msg[0]}`")
        #Stop loop after connect and publish successfully
        client.loop_stop()
        #Disconnect MQTT server
        client.disconnect()
    else:
         print(f"Publish failed to topic: `{msg[0]}`, trying to re-publish")

def subscribe(ID):              #return back the msg received
    #para usage: ID, msg
    #ID = [broker, port, topic, ID, username, password] có nên để topic vào không?
    #msg = {[info1],[info2],[info3]}

    #create client info with the input ID
    client = mqtt_client.Client(ID.client_id)
    client.username_pw_set(ID.username, ID.password)

    #check call back as a fail safe
    client.on_connect = on_connect

    #Trying to sub until fail (5 times)
    #set up client with the ID info
    client.connect(ID.broker_IP, ID.port)
    # Wait for connection to complete
    client.loop_start()
    #Sub to the broker and try to received msg
    result = client.subscribe('droneFB')
    client.on_message = on_message
    msgReceived = client.on_message
    if result[0] ==0:
        print(f"Received `{msgReceived}` from 'droneFB' topic")
        #Stop loop after connect and sub successfully
        client.loop_stop()
        #Disconnect MQTT server
        client.disconnect()
        return msgReceived
    else:
        print(f"Receive fail, trying to connect to 'droneFB' again")