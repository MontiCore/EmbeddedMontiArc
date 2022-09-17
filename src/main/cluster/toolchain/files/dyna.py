import os
import time
import sys
import pathlib
import subprocess

time_to_wait = 1200 # wait up to 20 minutes

this_directory = pathlib.Path(__file__).parent.resolve()
project_root = os.path.join(this_directory, '..', '..')
signal_file = os.path.join(this_directory, 'Lattice_Structures', 'signalfile') # This file comes from the local pc and marks the end of the copy process of 1.k
final_file = os.path.join(this_directory, 'Lattice_Structures', 'endfile') # This file is created by this script and indicates that lsdyna has finished

while True:
    time_counter = 0
    while not os.path.exists(signal_file): # Waiting for signalfile that indicates that 1.k has been copied to the cluster folder
        time.sleep(1)
        time_counter += 1
        if time_counter > time_to_wait:
            sys.exit("Local computer timed out!")
    subprocess.run(['make', 'dyna'], cwd=project_root)
    try:
        with open(final_file, 'x') as final: # Write 'endfile' as a signal that dyna has finished processing the file from the local pc
            pass
    except FileExistsError: # This should never happen, but if it happens, it means that the preprocessing did not correctly move and delete the 'endfile' from the previous iteration
        sys.exit("The preprocessing did not remove toolchain/files/Lattice_Structures/endfile ! (please check copy_preprocessing.py and dyna.py)")
