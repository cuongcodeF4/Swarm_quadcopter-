------------------------------------------------------------------------------
MQTT broker using Eclipse Mosquito protocol                                  
------------------------------------------------------------------------------

1. Description
--------------
This project use mqtt eclipse mosquito protocol to connect between publisher and subscriber via 
a broker, which name is eclipse Mosquito and it a broker run under the local network on a 
Raspberry Pi- It is a master machines. 

2. Setup environment 
----------------------
Eclipse mosquito exist in many envi, which make the set up phase more easy and applicable
    Linux: Most common choice, offering flexibility and stability.
    Windows: Simpler setup for less technical users.
    macOS: Similar to Linux in terms of installation and configuration.
    Docker: Containerized approach for portability and easier deployment across different environments.
----------------------
First, install the package needed for the envi for MQTT, follow the instruction on website: https://mosquitto.org/download/: https://mosquitto.org/download/ and download the version that fit you machine best.

After installation, locate the configuration file (mosquitto.conf) usually found in /etc/mosquitto (Linux) or C:\Program Files\mosquitto (Windows), or /opt/homebrew/etc/mosquitto/mosquitto.conf (for mac). Edit this file to configure settings like:
    Port number: Default is 1883, but you can change it if needed.
    >> you can do this by uncomment the port and port_tls lines and set them to the desire value (1883 for unencrypted connections and port 8883 for encrypted connections with TLS/SSL).
    User authentication: Enable and configure credentials if desired for secure access.
    >> This allow the client to connect anonymously by default
    >> do this by uncomment and configure the password_file or acl_file options to define usernames and passwords or access control rules.
    Persistence: Optionally enable message persistence to store messages even if the broker restarts.
    >> By default, all msg are lost when the broker restart. In order to retain the msg for later use uncomment and configure the persistence option to use a filesystem directory or database for storage.
    Other settings: Depending on your needs, adjust additional configurations like clustering, high availability, 
    or access control lists (ACLs)  .
    When finish, remember to back up a conf file in your system for later use if it ever needed.

if you install MQTT via home brew on mac, and the mosquitto line in the /opt/homebrew/etc/mosquitto/. That mean you're install the correct version of mosquitto.
if you install this on window check to see if the mosquitto folder in program files do contain mosquitto.conf file. If it does, it mean that you have install the correct version of mosquitto.
if you install MQTT via a package manager like apt on linux, search for "eclipse-mosquitto" or mentions "Eclipse Foundation" in the description. If yes then you have install the correct version of mosquitto in your system. 
whether you're using mosquitto or eclipse mosquitto, the code still stay the same so ne need to change anything 

#BEFORE CODE:
Install Python client library for MQTT on Windows: paho-mqtt
- pip install paho-mqtt
Install broker for local machine: Mosquito
- Search on browser: https://Mosquito.org/download/ , install and setup this software 
- Configure Mosquito in order to accept connections from remote clients. 
2.1. On your Windows machine, run a text editor as administrator and paste the following text:

listener 1883
allow_anonymous true

2.2.This creates a listener on port 1883 and allows anonymous connections. Save the file to 
"C:\Program Files\Mosquito" using a file name with the ".conf" extension such as "your_conf_file.conf".

2.3.Open a terminal window and navigate to the Mosquito directory. Run the following command:

Mosquito -v -c your_conf_file.conf

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
