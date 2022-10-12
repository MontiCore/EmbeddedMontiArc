import glob
import os
import pathlib
import shutil
import signal
import subprocess
import sys
import time

i = 0  # Iteration counter
delay = 2 # Delay (in seconds) between each iteration
current_path = pathlib.Path(__file__).parent.resolve()  # Project path


# Catch Ctrl-c
def signal_handler(signal, frame):
    print("Completed ", i, " iterations")
    print("exiting now...")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

print("Preparing...")

# Read number of iterations
iterations = int(input("Please input number of iterations (-1 for infinity):"))
if iterations == 0:
    print("0 is not a valid number of iterations")
    sys.exit()

## Iteration loop

while i < iterations:
    # Check for input files
    if not os.path.isfile(os.path.join(current_path, "files", "Input.csv")):
        sys.exit("You have not provided a Input.csv file for nTop!")

    # Python script for nTop
    print("Generating structure with nTop...")
    subprocess.run(['python', os.path.join(current_path, "tools", "Runiterations_AOB_8_Table_3.py")])

    # Modify and copy files
    subprocess.run(['python', os.path.join(current_path, "tools", "errorscript_Loop.py")])

    # Start LSDyna (Batch script)
    print("FEM-simulation...")
    subprocess.run([os.path.join(current_path, "files", "Lattice_Structures", "runcommand.bat")],
                   cwd=os.path.join(current_path, "files", "Lattice_Structures"))

    # Copy files
    print("Copying files for preprocessing...")
    lsdyna_files = glob.glob(os.path.join(current_path, "files", "Lattice_Structures", "*"))
    destination = os.path.join(current_path, "preprocessing", "raw", "run__01")
    filtered_lsdyna = [file for file in lsdyna_files if
                       not file.endswith("main.key") and not file.endswith("1000001.key") and not file.endswith(
                           "Control_cards.key") and not file.endswith("runcommand.bat") and not file.endswith(
                           "MAT_AA6014-T4_MAT24_kgmmms.key")]

    for file in filtered_lsdyna:
        path_to_file = os.path.join(current_path, "files", "Lattice_Structures", file)
        shutil.copy(path_to_file, destination)

    for file in filtered_lsdyna:
        path_to_file = os.path.join(current_path, "files", "Lattice_Structures", file)
        os.remove(path_to_file)

    path_to_inputcsv = os.path.join(current_path, "files", "Input.csv")
    shutil.move(path_to_inputcsv, os.path.join(current_path, "preprocessing", "raw"))

    # Start preprocessing container
    print("Preprocessing simulation data...")
    subprocess.run(['python', os.path.join(current_path, "tools", "container.py")])

    # Copy files (train.h5, train_graph)
    h5_files = glob.glob(os.path.join(current_path, "preprocessing", "h5", "raw", "*"))

    # RL-component
    print("Finished processing, starting RL-component...")
    print("ATTENTION: A local RL-component is not implemented yet!")

    # Clean up files
    print("Clean-up...")
    old_lsdyna = glob.glob(os.path.join(current_path, "preprocessing", "raw", "run__01", "*"))
    for file in old_lsdyna:
        path_to_file = os.path.join(current_path, "preprocessing", "raw", "run__01", file)
        os.remove(path_to_file)
    os.remove(os.path.join(current_path, "preprocessing", "raw", "Input.csv"))
    temp_path = os.path.join(current_path, "preprocessing", "lsdynabin_to_train", "__pycache__")
    shutil.rmtree(temp_path)

    i += 1
    print("Iteration " + str(i) + " finished!")
    print("Starting next iteration in " + str(delay) + " seconds...")
    time.sleep(delay)