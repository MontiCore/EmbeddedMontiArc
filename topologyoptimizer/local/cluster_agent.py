import csv
import logging
import os
import pathlib
import sys

from config import config
from local.tools.winscp import download_files_with_retry, upload_files
from local.tools.ntopology import generate_lattice
from local.tools import sbatch

# Initialize logging
logger = logging.getLogger(__name__)

def execute():
    current_path = pathlib.Path(__file__).parent.resolve()
    cluster_working_dir = config.get("DEFAULT", "ClusterWorkingDirectory")
    cluster_files_dir = f'{cluster_working_dir}/toolchain/files'
    cluster_rl_dir = f'{cluster_working_dir}/rl'
    cluster_lattice_structure_dir = f'{cluster_files_dir}/Lattice_Structures'
    local_files_dir = current_path.joinpath("files")
    local_lattice_structure_dir = local_files_dir.joinpath("Lattice_Structures")
    input_csv_filename = "Input.csv"
    constraint_filename = "constraint.txt"
    fe_mesh_filename = "1.k"
    signal_filename = "signalfile"
    local_input_csv_file = local_files_dir.joinpath(input_csv_filename)
    local_constraint_file = local_files_dir.joinpath(constraint_filename)
    local_fe_mesh_file = local_lattice_structure_dir.joinpath(fe_mesh_filename)

    print("Preparing...")
    logger.info("EXECUTE AGENT")

    constraint = int(input("Please input your minimum requirement for the energy absorption of the structure (1-100):"))
    if constraint == 0:
        print("0 is not a valid number for the energy absorption")
        logger.critical("Invalid constraint for energy absorption")
        sys.exit()

    with open(local_constraint_file, 'w') as file:
        file.write(str(constraint))
        file.close()

    print("Copying constraint to cluster...")
    logger.info("Copying constraint to cluster...")
    upload_files(constraint_filename, local_files_dir, cluster_rl_dir)

    ## Start Cluster
    print("Starting RL-component...")
    logger.info("Starting RL component...")
    sbatch.schedule_job("topologyoptimizer_agent", f'{cluster_working_dir}/agent.job')

    ## Wait for output file (Input.csv) from the cluster and copy it to the local folder
    logger.info("Waiting for output file (Input.csv) from cluster...")
    download_files_with_retry(input_csv_filename, local_files_dir, cluster_files_dir, delete_sources=True)

    # Check for input files
    if not os.path.isfile(local_input_csv_file):
        logger.critical("Cluster timeout")
        sys.exit("Cluster timeout. Exiting...")

    print("Received Input file for nTop!")
    logger.info("Input.csv received!")

    with open(local_input_csv_file, 'r') as inputcsv: # Print agent response
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
    upload_files( fe_mesh_filename, local_lattice_structure_dir, cluster_lattice_structure_dir)
    upload_files( signal_filename, local_lattice_structure_dir, cluster_lattice_structure_dir)

    # Clean up local files
    print("Clean-up...")
    logger.info("Clean-up files...")
    os.remove(local_input_csv_file)
    os.remove(local_fe_mesh_file)
    os.remove(local_constraint_file)
    logger.debug("cluster_agent.py finished")
