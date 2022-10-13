import csv
import os
import pathlib
import signal
import sys
import time

from config import config
from local.tools.ntopology import generate_lattice
from local.tools import winscp
from local.tools import sbatch

def train():
    i = 0  # Iteration counter
    delay = 20 # Delay (in seconds) between each iteration
    interrupted  = False # Flag to exit safely after Ctrl-c
    current_path = pathlib.Path(__file__).parent.resolve()  # Project path
    cluster_working_dir = config.get("DEFAULT", "ClusterWorkingDirectory")
    cluster_files_dir = f'{cluster_working_dir}/toolchain/files'
    cluster_lattice_structure_dir = f'{cluster_files_dir}/Lattice_Structures'
    local_files_dir = current_path.joinpath("files")
    local_lattice_structure_dir = local_files_dir.joinpath("Lattice_Structures")
    input_csv_filename = "Input.csv"
    fe_mesh_filename = "1.k"
    signal_filename = "signalfile"
    local_input_csv_file = local_files_dir.joinpath(input_csv_filename)
    local_fe_mesh_file = local_lattice_structure_dir.joinpath(fe_mesh_filename)

    print("Preparing...")

    # Catch Ctrl-c
    def signal_handler(signal, frame):
        print("Received stop signal, finishing last loop iteration...")
        global interrupted
        interrupted = True

    signal.signal(signal.SIGINT, signal_handler)

    ## Start Cluster
    print("Starting RL-component...")
    sbatch.schedule_job("topologyoptimizer_train", 'run.job')

    # Initial delay
    time.sleep(10)

    ## Iteration loop
    while True:
        if interrupted:
            print("Interrupted, completed ", i, " iterations")
            print("exiting now...")
            break

        winscp.download_files_with_retry([input_csv_filename], local_files_dir, cluster_files_dir, delete_sources=True)

        # Check for input files
        if not os.path.isfile(local_input_csv_file):
            sys.exit("Cluster timeout. Exiting...")

        print("Received Input file for nTop!")

        with open(local_input_csv_file, 'r') as inputcsv:
            reader = csv.reader(inputcsv)
            for row in reader:
                print(row)

        print("Generating structure with nTop...")
        generate_lattice(local_input_csv_file, local_fe_mesh_file)
        print("Finished!")
        
        # Copy necessary files to cluster
        print("Copying files to cluster...")
        winscp.upload_files( fe_mesh_filename, local_lattice_structure_dir, cluster_lattice_structure_dir)
        winscp.upload_files( signal_filename, local_lattice_structure_dir, cluster_lattice_structure_dir)

        # Clean up local files
        print("Clean-up...")
        os.remove(local_input_csv_file)
        os.remove(local_fe_mesh_file)

        i += 1
        print("Iteration " + str(i) + " finished!")
        print("Starting next iteration in " + str(delay) + " seconds...")
        time.sleep(delay)

def install():
    remote_cluster_dir = config.get("DEFAULT", "ClusterWorkingDirectory")
    local_cluster_dir = pathlib.Path(config.get("DEFAULT", "ProjectRootDirectory")).joinpath("src/main/cluster")

    if not winscp.exists(remote_cluster_dir):
        winscp.mkdir(remote_cluster_dir)
    else:    
        accept = str(input(f'{remote_cluster_dir} does exist on the cluster. Do you want to overwrite it? yes / no\n'))
        if accept != "yes":
            sys.exit(0)

    winscp.synchronize_directory(local_cluster_dir, remote_cluster_dir)
    