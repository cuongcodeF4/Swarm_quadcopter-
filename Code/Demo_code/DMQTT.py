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
def on_message(client, userdata, msg, received_messages, topic):                                  #Message receive call back
        try:
            received_messages[topic] = msg.payload.decode()
        except Exception as e:
            print(f"Error processing message from {topic}: {e}")


#only pub into droneFB topic to submit info to other channel
#with the ID pre-set with needed info base on what the master tell the drone to publish
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
    result = client.publish('droneFB', json.dumps(msg))
    #safe result for call back function
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

def subscribe(ID, timeout=10):
    """Subscribes to multiple topics and returns received messages within a timeout.

    Args:
        ID (object): An object containing relevant information like `client_id`, `username`, and `password`.
        timeout (int, optional): Maximum time to wait for messages in seconds. Defaults to 10.

    Returns:
        dict: A dictionary containing received messages with topics as keys, or None if timed out.
        e.g., {'sysCom': 'message content', 'droneCom': 'another message'}
    """

    subMaxRetry = 5
    availableTopic = ['sysCom', 'droneCom', 'conCom', 'droneFB']

    received_messages = {}

    for topic in availableTopic:
        # Create a new client for each topic for isolation
        client = mqtt_client.Client(f"{ID.client_id}-{topic}")
        client.username_pw_set(ID.username, ID.password)

        # Assign your existing on_message function to handle message reception
        client.on_message = on_message  # Use your provided on_message function

        # Connect and handle errors gracefully
        try:
            client.connect(ID.broker, ID.port)
            client.loop_start()
        except Exception as e:
            print(f"Error connecting to {topic}: {e}")
            continue  # Skip to the next topic if connection fails
        # Subscribe and handle retries with timeout
        result = client.subscribe(topic)
        subRetry = 0
        while result[0] != 0 and subRetry < subMaxRetry:
            try:
                result = client.subscribe(topic)
                time.sleep(0.5)  # Wait for potential asynchronous subscription response
            except Exception as e:
                print(f"Error subscribing to {topic}: {e}")
                subRetry += 1
                time.sleep(1)  # Wait before retrying
            if subRetry == subMaxRetry:
                print(f"Failed to subscribe to {topic} after {subRetry} retries")
        # Handle successful subscription and message reception
        if result[0] == 0:
            print(f"Subscribed to {topic}")
            # Wait for messages within the timeout
            start_time = time.time()
            while True:
                client.loop(2)  # Check for messages every 2 seconds
                if time.time() - start_time > timeout:
                    print(f"No messages received within {timeout} seconds")
                    break  # Break the loop if timed out
            # Disconnect (moved here for efficiency)
            client.loop_stop()
            client.disconnect()

    return received_messages if received_messages else None  # Return None if no messages received




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