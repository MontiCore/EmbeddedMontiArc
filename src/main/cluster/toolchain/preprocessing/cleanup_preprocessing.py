import glob
import os
import shutil


cluster_path = "/home/wj777230/Dokumente/topologyoptimizer"

print("Clean-up...")
old_lsdyna = glob.glob(os.path.join(cluster_path, "toolchain", "preprocessing", "raw", "run__01", "*"))
for file in old_lsdyna:
    path_to_file = os.path.join(cluster_path, "toolchain", "preprocessing", "raw", "run__01", file)
    os.remove(path_to_file)
os.remove(os.path.join(cluster_path, "toolchain", "preprocessing", "raw", "Input.csv"))
temp_path = os.path.join(cluster_path, "toolchain", "preprocessing", "lsdynabin_to_train", "__pycache__")
shutil.rmtree(temp_path)
