------------------------------------------------------------------------------
MQTT use broker Mosquitto                                   
------------------------------------------------------------------------------

1. Description
--------------
This project use mqtt protocol to connect between publisher and subscriber via 
a broker, which name is Mosquitto and it a broker run under the local on a 
Raspberry Pi- It is a master machines. 

2. Setup environment 
----------------------
Install Python client library for MQTT on Windows: paho-mqtt
- pip install paho-mqtt
Install broker for local machine: Mosquitto 
- Search on browser: https://mosquitto.org/download/ , install and setup this software 
- Configure mosquitto in order to accept connections from remote clients. 
2.1. On your Windows machine, run a text editor as administrator and paste the following text:

listener 1883
allow_anonymous true

2.2.This creates a listener on port 1883 and allows anonymous connections. Save the file to 
"C:\Program Files\Mosquitto" using a file name with the ".conf" extension such as "your_conf_file.conf".

2.3.Open a terminal window and navigate to the mosquitto directory. Run the following command:

mosquitto -v -c your_conf_file.conf

where

-c : specify the broker config file.

-v : verbose mode - enable all logging types. This overrides any logging options given in the config file.

Mac and Linux versions are also available on Internet.

3. Contact Info
---------------
If you have any questions, comments or suggestions, we would like to hear from
you.  For reporting bugs, you can contact to us by below link: 
    Email:  vietcuong2002s@gmail.com
    Zalo:   0399470672( Thái Việt Cường)
