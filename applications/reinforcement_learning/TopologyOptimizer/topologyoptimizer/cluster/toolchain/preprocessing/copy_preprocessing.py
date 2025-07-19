import glob
import logging
import os
from pathlib import Path
import shutil

working_dir = Path(os.environ['WORKING_DIR']).resolve()
log_folder = os.environ['LOG_FOLDER']

# Initialize logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
file_handler = logging.FileHandler(f'{log_folder}/preprocessing.log')
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)

logger.debug("Start copying files for preprocessing...")
print("Copying files for preprocessing...")
lsdyna_files = glob.glob(os.path.join(working_dir, "toolchain", "files", "Lattice_Structures", "*"))
destination = os.path.join(working_dir, "toolchain", "preprocessing", "raw", "run__01")
filtered_lsdyna = [file for file in lsdyna_files if
                   not file.endswith("main.key") and not file.endswith("1000001.key") and not file.endswith(
                       "Control_cards.key") and not file.endswith("runcommand.bat") and not file.endswith(
                       "MAT_AA6014-T4_MAT24_kgmmms.key")]

for file in filtered_lsdyna:
    path_to_file = os.path.join(working_dir, "toolchain", "files", "Lattice_Structures", file)
    shutil.copy(path_to_file, destination)

for file in filtered_lsdyna: # delete files from the origin
    path_to_file = os.path.join(working_dir, "toolchain", "files", "Lattice_Structures", file)
    os.remove(path_to_file)
logger.debug("Finished copying files for preprocessing")
