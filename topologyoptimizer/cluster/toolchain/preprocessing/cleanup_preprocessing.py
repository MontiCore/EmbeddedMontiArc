import glob
import os
from pathlib import Path
import shutil

working_dir = Path(os.environ['WORKING_DIR']).resolve()

print("Clean-up...")
old_lsdyna = glob.glob(os.path.join(working_dir, "toolchain", "preprocessing", "raw", "run__01", "*"))
for file in old_lsdyna:
    path_to_file = os.path.join(working_dir, "toolchain", "preprocessing", "raw", "run__01", file)
    os.remove(path_to_file)
os.remove(os.path.join(working_dir, "toolchain", "preprocessing", "raw", "Input.csv"))
temp_path = os.path.join(working_dir, "toolchain", "preprocessing", "lsdynabin_to_train", "__pycache__")
shutil.rmtree(temp_path)
