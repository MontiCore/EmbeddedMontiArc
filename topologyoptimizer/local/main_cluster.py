import csv
import logging
import os
import pathlib
import signal
import sys
import time

from config import config
from local.tools.ntopology import generate_lattice
from local.tools import winscp
from local.tools import sbatch
from local.tools import send_command

# Initialize logging
logger = logging.getLogger(__name__)

def train(options):
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
    logger.info("TRAIN AGENT")

    # Catch Ctrl-c
    def signal_handler(signal, frame):
        print("Received stop signal, finishing last loop iteration...")
        logger.info("Received stop signal, finishing last loop iteration...")
        global interrupted
        interrupted = True

    signal.signal(signal.SIGINT, signal_handler)

    ## Start Cluster
    print("Starting RL-component...")
    logger.info("Starting RL component...")
    sbatch.schedule_job("topologyoptimizer_train", 'run.job')

    # Initial delay
    time.sleep(10)

    ## Iteration loop
    while True:
        if interrupted:
            print("Interrupted, completed ", i, " iterations")
            print("exiting now...")
            logger.info("INTERRUPTED, completed %d iterations", i)
            break

        logger.info("Iteration %d started", i+1)
        logger.info("Waiting for output file (Input.csv) from cluster...")
        winscp.download_files_with_retry([input_csv_filename], local_files_dir, cluster_files_dir, delete_sources=True)

        # Check for input files
        if not os.path.isfile(local_input_csv_file):
            logger.critical("Cluster timeout")
            sys.exit("Cluster timeout. Exiting...")

        print("Received Input file for nTop!")
        logger.info("Input.csv received!")

        with open(local_input_csv_file, 'r') as inputcsv:
            reader = csv.reader(inputcsv)
            for row in reader:
                print(row)

        print("Generating structure with nTop...")
        logger.info("Generating structure with nTop...")
        generate_lattice(local_input_csv_file, local_fe_mesh_file)
        print("Finished!")
        logger.info("Finished")
        
        # Copy necessary files to cluster
        print("Copying files to cluster...")
        logger.info("Copying files to cluster...")
        winscp.upload_files( fe_mesh_filename, local_lattice_structure_dir, cluster_lattice_structure_dir)
        winscp.upload_files( signal_filename, local_lattice_structure_dir, cluster_lattice_structure_dir)

        # Clean up local files
        print("Clean-up...")
        logger.info("Clean-up files...")
        os.remove(local_input_csv_file)
        os.remove(local_fe_mesh_file)

        i += 1
        print("Iteration " + str(i) + " finished!")
        logger.info("Iteration %d finished", i)
        logger.info("----------------------------------------------------------------")
        print("Starting next iteration in " + str(delay) + " seconds...")
        time.sleep(delay)

def install(options):
    remote_cluster_dir = config.get("DEFAULT", "ClusterWorkingDirectory")
    local_cluster_dir = pathlib.Path(config.get("DEFAULT", "ProjectRootDirectory")).joinpath("topologyoptimizer/cluster")

    logger.info("INSTALL TOOLCHAIN ON CLUSTER")

    if not winscp.exists(remote_cluster_dir):
        winscp.mkdir(remote_cluster_dir)
    else:    
        accept = str(input(f'{remote_cluster_dir} does exist on the cluster. Do you want to overwrite it? yes / no\n'))
        if accept != "yes":
            sys.exit(0)

    logger.info("Copying files to cluster...")
    winscp.synchronize_directory(local_cluster_dir, remote_cluster_dir)
    send_command.send_command(f'find {remote_cluster_dir} -type f -name "*.sh" -print0 | xargs -0 chmod u+x')
    send_command.send_command(f'find {remote_cluster_dir} -type f -print0 | xargs -0 dos2unix')
    logger.info("Installing finished")
    