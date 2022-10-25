from asyncio import CancelledError
import csv
import os
import pathlib
import sys

from config import config
from local.tools.winscp import download_files_with_retry, upload_files
from local.tools.ntopology import generate_lattice
from local.tools import sbatch

async def execute():
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

    constraint = int(input("Please input your minimum requirement for the energy absorption of the structure (1-100):"))
    if constraint == 0:
        print("0 is not a valid number for the energy absorption")
        sys.exit()

    with open(local_constraint_file, 'w') as file:
        file.write(str(constraint))
        file.close()

    print("Copying constraint to cluster...")
    await upload_files(constraint_filename, local_files_dir, cluster_rl_dir)

    ## Start Cluster
    print("Starting RL-component...")
    sbatch.schedule_job("topologyoptimizer_agent", f'{cluster_working_dir}/agent.job')

    try:
        ## Wait for output file (Input.csv) from the cluster and copy it to the local folder
        await download_files_with_retry(input_csv_filename, local_files_dir, cluster_files_dir, delete_sources=True)
    except CancelledError:
        return

    # Check for input files
    if not os.path.isfile(local_input_csv_file):
        sys.exit("Cluster timeout. Exiting...")

    print("Received Input file for nTop!")

    with open(local_input_csv_file, 'r') as inputcsv: # Print agent response
        reader = csv.reader(inputcsv)
        for row in reader:
            print(row)

    print("Generating structure with nTop...")
    generate_lattice(local_input_csv_file, local_fe_mesh_file)
    print("Finished!")

    # Copy necessary files to cluster
    print("Copying files to cluster...")
    await upload_files( fe_mesh_filename, local_lattice_structure_dir, cluster_lattice_structure_dir)
    await upload_files( signal_filename, local_lattice_structure_dir, cluster_lattice_structure_dir)

    # Clean up local files
    print("Clean-up...")
    os.remove(local_input_csv_file)
    os.remove(local_fe_mesh_file)
    os.remove(local_constraint_file)
