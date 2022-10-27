from asyncio import CancelledError, sleep
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

async def train(options):
    i = 1  # Iteration counter
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

    logger.info("Start training")

    ## Start Cluster
    logger.info("Starting RL component...")
    sbatch.schedule_job("topologyoptimizer_train", 'run.job')

    # Initial delay
    await sleep(10)

    # Iteration loop
    try:
        while True:
            logger.info("Iteration %d started", i)
            logger.info("Fetch output file %s from cluster", input_csv_filename)
            await winscp.download_files_with_retry([input_csv_filename], local_files_dir, cluster_files_dir, delete_sources=True)

            # Check for input files
            if not os.path.isfile(local_input_csv_file):
                logger.critical("Cluster timeout. Exiting...")
                sys.exit(1)

            logger.info("Received output file %s from cluster", input_csv_filename)

            with open(local_input_csv_file, 'r') as inputcsv:
                reader = csv.reader(inputcsv)
                for row in reader:
                    print(row)

            logger.info("Structure generation with nTopology started...")
            generate_lattice(local_input_csv_file, local_fe_mesh_file)
            logger.info("Structure generation finished")
                
            logger.info("Upload files %s and %s to cluster...", fe_mesh_filename, signal_filename)
            await winscp.upload_files( fe_mesh_filename, local_lattice_structure_dir, cluster_lattice_structure_dir)
            await winscp.upload_files( signal_filename, local_lattice_structure_dir, cluster_lattice_structure_dir)
            logger.info("Upload finished")

            logger.info("Clean-up local files %s and %s", input_csv_filename, fe_mesh_filename)
            os.remove(local_input_csv_file)
            os.remove(local_fe_mesh_file)
            logger.info("Clean-up finished")

            logger.info("Iteration %d finished, starting next iteration in %d seconds", i, delay)
            logger.info("----------------------------------------------------------------")
            i += 1
            await sleep(delay)
    except CancelledError:
        logger.info("Interrupted, completed %d iterations", i-1)
        logger.info("Exiting now...")

async def install(options):
    remote_cluster_dir = config.get("DEFAULT", "ClusterWorkingDirectory")
    local_cluster_dir = pathlib.Path(config.get("DEFAULT", "ProjectRootDirectory")).joinpath("topologyoptimizer/cluster")

    logger.info("Installing software on cluster...")

    if not winscp.exists(remote_cluster_dir):
        try:
            winscp.mkdir(remote_cluster_dir)
        except Exception:
            logger.error("Could not create directory %s on the cluster. Do you have the right permissions?", remote_cluster_dir)
    else:    
        accept = str(input(f'{remote_cluster_dir} does exist on the cluster. Do you want to overwrite it? yes / no\n'))
        if accept != "yes":
            logging.info("Aborting install...")
            sys.exit(0)

    logger.info("Copy project files to cluster...")
    await winscp.synchronize_directory(local_cluster_dir, remote_cluster_dir)
    logger.info("Copy project files to cluster finished!")
    
    try:
        logger.info("Set script permissions...")
        send_command.send_command(f'find {remote_cluster_dir} -type f -name "*.sh" -print0 | xargs -0 chmod u+x')
    except:
        logger.exception("Failed to set script permissions")
        sys.exit(1)
    
    try:
        logger.info("Replace windows \\r\\n line endings....")
        send_command.send_command(f'find {remote_cluster_dir} -type f -print0 | xargs -0 dos2unix')
    except:
        logger.exception("Failed to replace windows \\r\\n line endings")
        sys.exit(1)
    logger.info("Installing software on cluster finished")
    