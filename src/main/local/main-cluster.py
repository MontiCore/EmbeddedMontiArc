import configparser
import csv
import os
import pathlib
import signal
import subprocess
import sys
import time

i = 0  # Iteration counter
delay = 20 # Delay (in seconds) between each iteration
interrupted  = False # Flag to exit safely after Ctrl-c
current_path = pathlib.Path(__file__).parent.resolve()  # Project path
config = configparser.ConfigParser() # Reading config
config.read('../../../config.ini')
config = config['DEFAULT']
cluster_user = config['ClusterUser']  # Cluster username for cluster computation
cluster_key_password = config['ClusterKeyPassword'] # Password for ssh key for cluster authentication
cluster_path = config['ClusterWorkingDirectory'] # Cluster path

print("Preparing...")

# Catch Ctrl-c
def signal_handler(signal, frame):
    print("Received stop signal, finishing last loop iteration...")
    global interrupted
    interrupted = True

signal.signal(signal.SIGINT, signal_handler)

## Start Cluster
print("Starting RL-component...")
subprocess.run(['python', os.path.join(current_path, "tools", "cluster.py"), current_path, cluster_user, cluster_key_password, cluster_path])

# Initial delay
time.sleep(10)

## Iteration loop
while True:
    if interrupted:
        print("Interrupted, completed ", i, " iterations")
        print("exiting now...")
        break

    ## Wait for output file (Input.csv) from the cluster and copy it to the local folder
    subprocess.run([os.path.join(current_path, "tools", "Cluster_to_PC.bat"),
         cluster_user,                                               # Cluster username
         os.path.join(current_path, "tools", "Keys", "key.ppk"),     # Path to ssh key
         cluster_key_password,                                       # Password for ssh key
         os.path.join(current_path, "files"),                        # Local path (destination)
         '/home/'+ cluster_user +'/'+ cluster_path +'/src/main/cluster/toolchain/files'],     # Path on the cluster
             cwd=os.path.join(current_path, "tools"))

    # Check for input files
    if not os.path.isfile(os.path.join(current_path, "files", "Input.csv")):
        sys.exit("Cluster timeout. Exiting...")

    print("Received Input file for nTop!")

    with open(os.path.join(current_path, "files", "Input.csv"), 'r') as inputcsv:
        reader = csv.reader(inputcsv)
        for row in reader:
            print(row)

    # Python script for nTop
    print("Generating structure with nTop...")
    subprocess.run(['python', os.path.join(current_path, "tools", "Runiterations_AOB_8_Table_3.py")])

    # Modify and copy files
    subprocess.run(['python', os.path.join(current_path, "tools", "errorscript_Loop.py")])
    print("Finished!")

    # Copy necessary files to cluster
    print("Copying files to cluster...")
    subprocess.run(
        ['winscp.com', '/ini=nul', '/log=transferP2C.log', '/script=tools/PC_to_Cluster.txt', '/parameter',
         cluster_user,                                                      # Cluster username
         os.path.join(current_path, "tools", "Keys", "key.ppk"),            # Path to ssh key
         cluster_key_password,                                              # Password for ssh key
         os.path.join(current_path, "files", "Lattice_Structures"),         # Local path (origin)
         '/home/'+ cluster_user +'/'+ cluster_path +'/src/main/cluster/toolchain/files/Lattice_Structures'])     # Path on the cluster

    # Clean up local files
    print("Clean-up...")
    os.remove(os.path.join(current_path, "files", "Input.csv"))
    os.remove(os.path.join(current_path, "files", "Lattice_Structures", "1.k"))

    i += 1
    print("Iteration " + str(i) + " finished!")
    print("Starting next iteration in " + str(delay) + " seconds...")
    time.sleep(delay)
