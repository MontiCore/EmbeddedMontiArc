import logging
import os
import time
import sys
import pathlib
import subprocess

time_to_wait = 4800 # wait up to 80 minutes

this_directory = pathlib.Path(__file__).parent.resolve()
project_root = os.path.join(this_directory, '..', '..')
csv_file = os.path.join(this_directory, 'Input.csv')
signal_file = os.path.join(this_directory, 'Lattice_Structures', 'signalfile') # This file comes from the local pc and marks the end of the copy process of 1.k
final_file = os.path.join(this_directory, 'Lattice_Structures', 'endfile') # This file is created by this script and indicates that lsdyna has finished

log_folder = os.environ['LOG_FOLDER']

# Initialize logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
file_handler = logging.FileHandler(f'{log_folder}/rosgym.log')
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)

while True:
    logger.debug("Start dyna.py")
    print("Start dyna.py")
    time_counter = 0
    while os.path.exists(csv_file) or not os.path.exists(signal_file): # Waiting for local pc to download and delete the input file
        time.sleep(1)
        time_counter += 1
        if time_counter > time_to_wait:
            logger.critical("Local computer times out")
            sys.exit("Local computer timed out!")
    print("Received 1.k")
    logger.debug("Received 1.k")
    logger.debug("Waiting for Dyna to finish simulation...")
    subprocess.run(['sh', './dyna.sh'], cwd=this_directory)
    if os.path.exists(signal_file) or os.path.exists(csv_file):
        logger.critical("Signal file existed although it should be deleted")
        sys.exit("This should never happen!")
    logger.debug("Dyna has finished")
    print("Dyna has finished")
    try:
        with open(final_file, 'x') as final: # Write 'endfile' as a signal that dyna has finished processing the file from the local pc
            pass
        print("endfile written")
        logger.debug("Endfile written")
    except FileExistsError: # This should never happen, but if it happens, it means that the preprocessing did not correctly move and delete the 'endfile' from the previous iteration
        logger.exception("The preprocessing did not remove toolchain/files/Lattice_Structures/endfile ! (please check copy_preprocessing.py and dyna.py)")
        sys.exit("The preprocessing did not remove toolchain/files/Lattice_Structures/endfile ! (please check copy_preprocessing.py and dyna.py)")
