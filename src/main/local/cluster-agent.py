import csv
import os
import pathlib
import subprocess
import sys

interrupted  = False # Flag to exit safely after Ctrl-c
current_path = pathlib.Path().resolve()  # Project path
cluster_user = "wj777230"  # Cluster username for cluster computation
cluster_key_password = 'Pax40nO9b' # Password for ssh key for cluster authentication
constraint_file = os.path.join(current_path, "tools", "constraint.txt")

print("Preparing...")

constraint = int(input("Please input your minimum requirement for the energy absorption of the structure (1-100):"))
if constraint == 0:
    print("0 is not a valid number for the energy absorption")
    sys.exit()

with open(constraint_file, 'w') as file:
    file.write(str(constraint))
    file.close()

print("Copying constraint to cluster...")
subprocess.run(
    ['winscp.com', '/ini=nul', '/script=tools/Constraint_to_Cluster.txt', '/parameter',
     cluster_user,                                                      # Cluster username
     os.path.join(current_path, "tools", "Keys", "key.ppk"),            # Path to ssh key
     cluster_key_password,                                              # Password for ssh key
     os.path.join(current_path, "tools"),         # Local path (origin)
     '/home/wj777230/Dokumente/topologyoptimizer/rl'])     # Path on the cluster

## Start Cluster
print("Starting RL-component...")
subprocess.run(['python', os.path.join(current_path, "tools", "agent.py"), current_path, cluster_user])

## Wait for output file (Input.csv) from the cluster and copy it to the local folder
subprocess.run([os.path.join(current_path, "tools", "Cluster_to_PC.bat"),
     cluster_user,                                                      # Cluster username
     os.path.join(current_path, "tools", "Keys", "key.ppk"),            # Path to ssh key
     cluster_key_password,                                                       # Password for ssh key
     os.path.join(current_path, "files"),                               # Local path (destination)
     '/home/wj777230/Dokumente/topologyoptimizer/toolchain/files'],     # Path on the cluster
         cwd=os.path.join(current_path, "tools"))

# Check for input files
if not os.path.isfile(os.path.join(current_path, "files", "Input.csv")):
    sys.exit("Cluster timeout. Exiting...")

print("Received Input file for nTop!")

with open(os.path.join(current_path, "files", "Input.csv"), 'r') as inputcsv: # Print agent response
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
     '/home/wj777230/Dokumente/topologyoptimizer/toolchain/files/Lattice_Structures'])     # Path on the cluster

# Clean up local files
print("Clean-up...")
os.remove(os.path.join(current_path, "files", "Input.csv"))
os.remove(os.path.join(current_path, "files", "Lattice_Structures", "1.k"))
os.remove(os.path.join(current_path, "tools", "constraint.txt"))
