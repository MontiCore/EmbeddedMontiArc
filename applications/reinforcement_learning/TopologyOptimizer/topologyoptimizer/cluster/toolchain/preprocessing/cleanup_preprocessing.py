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

logger.debug("Start Clean-up...")
print("Clean-up...")
old_lsdyna = glob.glob(os.path.join(working_dir, "toolchain", "preprocessing", "raw", "run__01", "*"))
for file in old_lsdyna:
    path_to_file = os.path.join(working_dir, "toolchain", "preprocessing", "raw", "run__01", file)
    os.remove(path_to_file)
os.remove(os.path.join(working_dir, "toolchain", "preprocessing", "raw", "Input.csv"))
temp_path = os.path.join(working_dir, "toolchain", "preprocessing", "lsdynabin_to_train", "__pycache__")
shutil.rmtree(temp_path)
logger.debug("Finished Clean-up")
logger.debug("--------------------------------")
