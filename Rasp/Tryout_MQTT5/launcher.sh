# #!/bin/sh
# # launcher.sh
# # navigate to home directory, then to this directory, then execute python script, then back home

# check_internet() {
#     while true; do
#         if ping -c 1 8.8.8.8 > /dev/null 2>&1; then
#             echo "Internet connected"
#             break
#         else
#             echo "Waiting for internet connection..."
#             sleep 3
#         fi
#     done
# }

# cd /
# . /home/pi/.venv/bin/activate
# cd /home/pi/Mechatronics_Project/Swarm_quadcopter-/MQTT/Tryout_MQTT5/
# /home/pi/.venv/bin/python Drone1.py
# deactivate
# cd /

