import csv
import os
import pathlib
import signal
import subprocess
import sys
import time

from local.tools.winscp import download_files_with_retry, upload_files,  download_files
from config import config

def train():
    i = 0  # Iteration counter
    delay = 20 # Delay (in seconds) between each iteration
    interrupted  = False # Flag to exit safely after Ctrl-c
    current_path = pathlib.Path(__file__).parent.resolve()  # Project path
    cluster_user = config.get("DEFAULT", "ClusterUser")
    cluster_key_password = config.get("DEFAULT", "ClusterKeyPassword")
    ssh_key = pathlib.Path(config.get("DEFAULT", "SSHKeyFolder")).joinpath("key.priv")
    cluster_home_dir = f'/home/{cluster_user}'
    cluster_working_dir = config.get("DEFAULT", "ClusterWorkingDirectory")
    cluster_files_dir = f'{cluster_home_dir}/{cluster_working_dir}/toolchain/files'
    local_files_dir = current_path.joinpath("files")
    input_csv_filename = "Input.csv"
    fe_mesh_filename = "1.k"
    signalfile_filename = "signalfile"

    print("Preparing...")

    # Catch Ctrl-c
    def signal_handler(signal, frame):
        print("Received stop signal, finishing last loop iteration...")
        global interrupted
        interrupted = True

    signal.signal(signal.SIGINT, signal_handler)

    ## Start Cluster
    print("Starting RL-component...")
    subprocess.run(['python', os.path.join(current_path, "tools", "cluster.py"), current_path, cluster_user, cluster_key_password, ssh_key])

    # Initial delay
    time.sleep(10)

    ## Iteration loop
    while True:
        if interrupted:
            print("Interrupted, completed ", i, " iterations")
            print("exiting now...")
            break

        download_files_with_retry([input_csv_filename], local_files_dir, cluster_files_dir, delete_sources=True)

        # Check for input files
        if not os.path.isfile(local_files_dir.joinpath(input_csv_filename)):
            sys.exit("Cluster timeout. Exiting...")

        print("Received Input file for nTop!")

        with open(local_files_dir.joinpath(input_csv_filename), 'r') as inputcsv:
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
        upload_files( fe_mesh_filename, local_files_dir.joinpath("Lattice_Structures"), f'{cluster_files_dir}/Lattice_Structures')
        upload_files( signalfile_filename, local_files_dir.joinpath("Lattice_Structures"), f'{cluster_files_dir}/Lattice_Structures')

        # Clean up local files
        print("Clean-up...")
        os.remove(local_files_dir.joinpath(input_csv_filename))
        os.remove(local_files_dir.joinpath(f'Lattice_Structures/{fe_mesh_filename}'))

        i += 1
        print("Iteration " + str(i) + " finished!")
        print("Starting next iteration in " + str(delay) + " seconds...")
        time.sleep(delay)
