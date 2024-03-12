import csv
import glob
import subprocess
import time
import os, signal

# Define DroneConfig class
class DroneConfig:
    def __init__(self):
        self.hw_id = self.get_hw_id()
        self.trigger_time = 0
        self.config = self.read_config()
        self.config['state'] = 0

    def get_hw_id(self):
        # Check the files in the current directory and find the hwID file
        hw_id_files = glob.glob("*.hwID")
        if hw_id_files:
            hw_id_file = hw_id_files[0]
            print(f"Hardware ID file found: {hw_id_file}")
            # Return the hardware ID without the extension (.hwID)
            hw_id = hw_id_file.split(".")[0]
            print(f"Hardware ID: {hw_id}")
            return hw_id
        else:
            print("Hardware ID file not found. Please check your files.")
            print("Exiting the program...")
            exit()

    def read_config(self):
        # Read the configuration from the CSV file
        with open('config.csv', newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                if row['hw_id'] == self.hw_id:
                    print(f"Configuration for HW_ID {self.hw_id} found in local CSV file.")
                    return row
            # If the HW_ID is not found in the CSV file, exit the program
            print(f"Configuration for HW_ID {self.hw_id} not found in local CSV file.")
            print("Exiting the program...")
            exit()

# Function to initialize MAVLink routing
def initialize_mavlink():
    endpoints = []
    endpoints.append(f"-e {drone_config.config['gcs_ip']}:{drone_config.config['gcs_port']}")
    endpoints.append(f"-p {drone_config.config['sitl_ip']}:{drone_config.config['sitl_port']}")
    # print(f"Endpoints: {endpoints}") # Debugging
    mavrouterd_cmd = "mavlink-routerd " + " ".join(endpoints)
    print("Running mavlink-routerd...")
    mavlink_router_process = subprocess.Popen(mavrouterd_cmd, shell=True, preexec_fn=os.setsid)
    return mavlink_router_process

# Function to stop MAVLink routing
def stop_mavlink_routing(mavlink_router_process):
    if mavlink_router_process:
        print("Stopping MAVLink routing...")
        os.killpg(os.getpgid(mavlink_router_process.pid), signal.SIGTERM)
        mavlink_router_process = None
    else:
        print("MAVLink routing is not running.")

def main():
    print("Start the coordinator")

    try:
        # Initialize mavlink
        print("Initialize mavlink...")
        # print(drone_config.config['gcs_ip'])
        mavlink_router_process = initialize_mavlink()
    
        while (True):
            try:
                pass
            except KeyboardInterrupt:
                print("Keyboard interrupt detected.")
                stop_mavlink_routing(mavlink_router_process)
                break
    
    except Exception as e:
        print("An error occurred: ", e)
    finally:
        pass

    print("End the coordinator")

if __name__ == "__main__":
    drone_config = DroneConfig()
    main()