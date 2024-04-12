import paho.mqtt.client as paho
import time
import paho.mqtt.properties as props
import paho.mqtt.packettypes as packetTypes
SYS_COM = "topicSystemCom"
DRONE_COM = "topicDroneCom"
CONTROL_COM = "topicControlCom"

def on_publish(client, userdata, mid):
    print("Message ID:", mid)

client = paho.Client(protocol=5)
client.on_publish = on_publish

# Connect to the MQTT broker
client.connect('mqtt.eclipseprojects.io', 1883)

# Your custom properties (user-defined)
# Your custom properties (user-defined)
custom_properties = props.Properties( packetTypes.PacketTypes.WILLMESSAGE)
custom_properties.UserProperty = [('Test', "ON")]
# Keep the client loop running
client.loop_start()
# Publish a message with properties
client.publish(DRONE_COM, payload='Hello, MQTT!', qos=2, properties=custom_properties)
# Keep the client loop running
client.loop_stop()

