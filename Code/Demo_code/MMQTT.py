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

droneNum = 0        # assign for the number of drone in the pack 

def on_connect(client, userdata, flags, rc):                    #Connection call back, use to check wether the connection is good or not
    #check ing call back value
    if rc == 0:
        #call back connected, push out msg and print successfully
        print("Connected to MQTT Broker!")
    else:
        #opposite value to connect successfully
        print("Failed to connect, return code %d\n", rc)
def on_message(client, userdata, msg, received_messages, topic):                          #Message receive call back
        #OLD FORMAT
        #msg_recv = msg.payload.decode()
        #msg_recv_dict = json.loads(msg_recv)
        #msgReceive = msg_recv_dict["drone1"]
        #return msgReceive
        
        #TEST FORMAT
        try:
            received_messages[topic] = msg.payload.decode()
        except Exception as e:
            print(f"Error processing message from {topic}: {e}")

def getCommand():

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


def publish(ID,topic, msg):                                                #Publish function, msg is in dict format
    #para usage: ID, msg
    #ID = [broker, port,...] có nên để topic vào không?


    #take in message
    #msg = getPub_msg()                                     #msg = ["topic", msg] or ["topic",["command","value"]]
    limit_trial = 5

    #create client info with the input ID
    client = mqtt_client.Client()

    #check call back as a fail safe
    client.on_connect = on_connect
    #Trying to publish until fail (5 time)

    #try to connect to the broker
    try:
        client.connect(ID.broker, ID.port)
        client.loop_start()
    except Exception as e:
        print(f"Error connecting to {topic}: {e}")
    #create a time out and time limit for easy debug
    trial = 1
    
    while (trial < limit_trial):
        # Publish the dictionary (converted to JSON)
        result = client.publish(topic, json.dumps(msg))
        #safe result for call back function
        #result format is 0,1 and 2 with 0 indicate for successful publish
        #call back when success publish msg
        if result[0]==0:
            print(f"Send `{msg}` to topic:`{topic}`")
            #Stop loop after connect and publish successfully
            client.loop_stop()
            #Disconnect MQTT server
            client.disconnect()
        else:
            trial += 1
            print(f"Publish failed to topic: `{topic}`, trying to re-publish, trial: {trial}")
    #fail to pub and exceed the trial limit
    if trial > limit_trial:
        #print fail to pub
        print("TIME OUT FOR PUBLISH!")
        print(f"Fail to publish to topic '{topic}' ")
        #reset time out sequence
        trial = 1
        #end the connect loop
        client.loop_stop()
        client.disconnect()

def subscribe(ID):                                              #return back the msg received
    #para usage: ID, msg
    #ID = [broker, port, topic, ID, username, password] có nên để topic vào không?
    #msg = {[info1],[info2],[info3]}

    #sub limit trial, 5 times
    limit_trial = 5

    received_messages = {}

    #create client info with the input ID
    client = mqtt_client.Client(ID.client_id)
    client.username_pw_set(ID.username, ID.password)

    #check call back as a fail safe
    client.on_connect = on_connect

    #Trying to sub until fail (5 times)
    #set up client with the ID info
    try:
        client.connect(ID.broker, ID.port)
        client.loop_start()
    except Exception as e:
        print(f"Error connecting to droneFB : {e}")
    
    trial = 0
    while trial < limit_trial:
        #Sub to the broker and try to received msg
        result = client.subscribe('droneFB')
        client.on_message = on_message
        msgReceived = client.on_message
        if result[0] ==0:
            print(f"Received message from 'droneFB' topic")
            #Stop loop after connect and sub successfully
            client.loop_stop()
            #Disconnect MQTT server
            client.disconnect()
        else:
            trial += 1 
            print(f"Receive fail, trying to connect to 'droneFB' again")
    #fail to pub and exceed the trial limit
    if trial > limit_trial:
        #print fail to pub
        print("TIME OUT FOR SUBSCRIBE")
        print("Fail to publish to Feed back thread")
        #reset time out sequence
        trial = 0
        #end the connect loop
        client.loop_stop()
        client.disconnect()
    #sum uo the msg and out put
    if received_messages:
        msg = received_messages
    else:
        pass
    return   msg# Return None if no messages received

def decode(command):

    pass

def comCheckUp(ID):                                               #Check up the connection between drone and master so that we know if they can send info to each other                            
    #set up as a normal publish func
    #but with the msg as a command that send to all the topic to test all the msg
    #when the master pub up the drone should answer with format "CP_droneNum" }
    #scan the string and do a check list to see if the master receive enough drone nums feed back
    msg = "comCheckUp"
    test_topic = ['sysCom', 'droneCom', 'conCom']
    for topic in test_topic:
        #pub to all the topic to see what happen
        publish(ID, topic, msg)
        #start counting down the time 
        timeBegin = time.time()
        #wait for a bit for the drone to pub there answer
        temp_msg_stack = []
        while timeout < 2:
            #get the msg from the droneFB channel
            msg = subscribe(ID)
            if msg in temp_msg_stack:
                #skip if the msg is the same
                #fail safe if duplicate ever occur
                pass
            else:
                #add in pack for later counting
                temp_msg_stack.extend(msg)
            #timeout function
            timeEnd = time.time() 
            timeout = timeEnd - timeBegin 
        #Scan the msg to count out the amount of drone that is responsive
        droneNumCounted = len(temp_msg_stack)
        if droneNum == droneNumCounted:
            print(f"comCheckUp on '{topic}' successful")
            print("Continue to the next topic")
        else:
            successDroneNum = ""
            #report back the fail drone to the terminal so that the user know
            #scan for all the success drone
            for drone_msg in temp_msg_stack:
                successDroneNum += ", " + drone_msg.split('_')[-1]
            print(f"Failure comCheckUp on '{topic}'")
            print("Success connection on Drone: %s", successDroneNum)
            print("Continue to the next topic")

def posReport(ID):
    #send the command to the sysCom topic to announce for the drone
    publish(ID, "sysCom", "posReport")
    time.sleep(1)
    #the return data will be a dict consist of {Num, lon, lat, alt}
    msg  = subscribe(ID)
    pass

def execute(ID, command):
    #Run the function needed for the command selected
    #Need to scan and take in the command format with the right data format
    #take out the topic data
    topic = command[0]
    msg  = command[1]
    available_topic = ['sysCom', 'droneCom', 'conCom']
    #create three command pack for the three topic 

    sysCom_commandPack = {
        "comCheckUp" : "func1",  
        "posReport" : "func2",
        "sysReport" : "func 3",
        "setGPSOrigin" : "func 4"
    }
    droneCom_commandPack = {
        "tunning" : "func 1",
        "arm" : "func2",
        "takeoff" : "func 3",
        "land" : "func 4",
        "unitTest" :"func 5"
    }
    conCom_commandPack = {
        "arm" : "func 1",
        "takeoff" : "func 2",
        "land" : "func 3",
        "init" : "func 4"
    }
    #scan the data to see what to excuse
    if topic == "sysCom":
        #Using the msg output as and atribute to run all funcs without any long if else chang
        sysCom_commandPack[msg]() 
        pass
    elif topic == "droneCom":
        droneCom_commandPack[msg]()
        pass
    else:
        conCom_commandPack[msg]()
        pass
    #scan the msg to see which func need to be init or run

    pass
    return None