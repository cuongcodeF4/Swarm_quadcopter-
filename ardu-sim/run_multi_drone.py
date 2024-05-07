import subprocess

# Kill all screen sessions
subprocess.run("killall screen", shell=True)

# Create new screen session for drone1
subprocess.run("screen -S drone1 -d -m ~/Swarm_quadcopter-/ardu-sim/arducopter -S --model + --speedup 1 --defaults copter.parm -I0 --home -35.363261,149.16523,584.0,0 --sysid=1", shell=True)
subprocess.run("screen -S mavlink_routerd1 -d -m mavlink-routerd -e 127.0.0.1:14550 -p 127.0.0.1:5760", shell=True)

# Create new screen session for drone2
subprocess.run("screen -S drone2 -d -m ~/Swarm_quadcopter-/ardu-sim/arducopter -S --model + --speedup 1 --defaults copter.parm -I1 --home -35.3635304961486,149.16523,584.0,0 --sysid=2", shell=True)
subprocess.run("screen -S mavlink_routerd2 -d -m mavlink-routerd -e 127.0.0.1:14560 -p 127.0.0.1:5770", shell=True)

# List all screen sessions
subprocess.run("screen -ls", shell=True)