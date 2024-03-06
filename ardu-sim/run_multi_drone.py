import subprocess
import time

# Kill all screen sessions
subprocess.run("killall screen", shell=True)

# Create new screen session for drone1
subprocess.run("screen -S drone1 -d -m ./arducopter -S --model + --speedup 1 --defaults parameters/copter.parm -I0", shell=True)
subprocess.run("screen -S mavproxy1 -d -m mavproxy.py --master tcp:127.0.0.1:5760 --out 127.0.0.1:14550", shell=True)

# Create new screen session for drone2
subprocess.run("screen -S drone2 -d -m ./arducopter -S --model + --speedup 1 --defaults parameters/copter.parm -I1 --home -35.363426,149.165252,584,353", shell=True)
subprocess.run("screen -S mavproxy2 -d -m mavproxy.py --master tcp:127.0.0.1:5770 --out 127.0.0.1:14560", shell=True)

