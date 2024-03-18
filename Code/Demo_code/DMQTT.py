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


#SIDE PARAMETER
IsInControlled = False


def get_msg(sub_msgReceived):
    return None



def on_connect(client, userdata, flags, rc):                    #Connection call back, use to check wether the connection is good or not
    #check ing call back value
    if rc == 0:
        #call back connected, push out msg and print successfully
        print("Connected to MQTT Broker!")
    else:
        #opposite value to connect successfully
        print("Failed to connect, return code %d\n", rc)
def on_message(client, userdata, msg, received_messages, topic):                                 #Message receive call back
        #OLD FORMAT
        #msg_recv = msg.payload.decode()
        #msg_recv_dict = json.loads(msg_recv)
        #msgReceive = msg_recv_dict["drone1"]
        #return msgReceive
        
        try:
            received_messages[topic] = msg.payload.decode()
        except Exception as e:
            print(f"Error processing message from {topic}: {e}")


#only pub into droneFB topic to submit info to other channel
#with the ID pre-set with needed info base on what the master tell the drone to publish
def publish(ID, msg):   
    #limit trial times
    limit_trial = 5 

    #create client info with the input ID
    client = mqtt_client.Client()

    #check call back as a fail safe
    client.on_connect = on_connect
    #Trying to publish until fail (5 time)

    #set up client with the ID info
    client.connect(ID.broker_IP, ID.port)
    # Wait for connection to complete
    client.loop_start()
    #Add on trial time, five times trial
    trial = 0
    while trial < limit_trial:
        # Publish the empty dictionary (converted to JSON)
        #safe result for call back function
        result = client.publish('droneFB', json.dumps(msg))
        #result format is 0,1 and 2 with 0 indicate for successful publish
        #call back when success publish msg
        if result[0]==0:
            print(f"Send `{msg}` to topic: 'droneFB'")
            #Stop loop after connect and publish successfully
            client.loop_stop()
            #Disconnect MQTT server
            client.disconnect()
        else:
            print(f"Publish failed to topic: 'droneFB', trying to re-publish")
            trial += 1
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

def subscribe(ID, topic):
    # limit try time, five times
    limit_trial = 5

    # Create a empty dict to store the msg
    received_messages = {}

    
    # Create a new client for each topic for isolation
    client = mqtt_client.Client(f"{ID.client_id}-{topic}")
    client.username_pw_set(ID.username, ID.password)

    # Assign your existing on_message function to handle message reception
    client.on_message = on_message  # Use your provided on_message function

    # Connect and handle errors gracefully
    #try to connect to the broker
    try:
        client.connect(ID.broker, ID.port)
        client.loop_start()
    except Exception as e:
        print(f"Error connecting to {topic}: {e}")

    # Subscribe and handle retries with timeout
    result = client.subscribe(topic)
    trial = 0 
    while trial < limit_trial:
        #Sub to the broker and try to received msg
        result = client.subscribe(topic)
        client.on_message = on_message
        if result[0] ==0:
            print(f"Received message from '{topic}' topic")
            #Stop loop after connect and sub successfully
            client.loop_stop()
            #Disconnect MQTT server
            client.disconnect()
        else:
            trial += 1 
            print(f"Receive fail, trying to connect to '{topic}' again")
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
    
    if received_messages:
        msg = [topic, received_messages]
    else:
        pass
    return   msg            # Return None if no messages received



    # while result[0] != 0 and subRetry < subMaxRetry:
    #     try:
    #         result = client.subscribe(topic)
    #         time.sleep(0.5)  # Wait for potential asynchronous subscription response
    #     except Exception as e:
    #         print(f"Error subscribing to {topic}: {e}")
    #         subRetry += 1
    #         time.sleep(1)  # Wait before retrying
    #     if subRetry == subMaxRetry:
    #         print(f"Failed to subscribe to {topic} after {subRetry} retries")
    # # Handle successful subscription and message reception
    # if result[0] == 0:
    #     print(f"Subscribed to {topic}")
    #     # Wait for messages within the timeout
    #     start_time = time.time()
    #     while True:
    #         client.loop(2)  # Check for messages every 2 seconds
    #         if time.time() - start_time > timeout:
    #             print(f"No messages received within {timeout} seconds")
    #             break  # Break the loop if timed out
    #     # Disconnect (moved here for efficiency)
    #     client.loop_stop()
    #     client.disconnect()
    

#thuc hien cau lenh nhan duoc sau khi chạy hàm pub 
def execute(command_pack):
    #available command
    commandAvailable = [
        'comCheckUp',

        #add in command for the three topic 
    ]
    if command_pack[1] in commandAvailable: 
        #isolate the droneCOom and conCom
        if command_pack[0] == "droneCom" and IsInControlled:
            pass
        elif command_pack[0] == "sysCom":

            pass
        else:
            pass
    else:
        print("in sufficient data or command input")

#cần add them para msg, có các trường hợp sẽ pub ra cho router, làm. một hàm duyệt tâts cả các biến
# được thay đổi và chỉ cho những biến đó được hiển thị trong msg khi pub
# forma khi chỉnh có thể là parameter.msg.<inf_id> = <value>
def comCheckUp_func():

    pass