import glob
import os
import shutil


cluster_path = "/home/wj777230/Dokumente/topologyoptimizer"


print("Copying files for preprocessing...")
lsdyna_files = glob.glob(os.path.join(cluster_path, "toolchain", "files", "Lattice_Structures", "*"))
destination = os.path.join(cluster_path, "toolchain", "preprocessing", "raw", "run__01")
filtered_lsdyna = [file for file in lsdyna_files if
                   not file.endswith("main.key") and not file.endswith("1000001.key") and not file.endswith(
                       "Control_cards.key") and not file.endswith("runcommand.bat") and not file.endswith(
                       "MAT_AA6014-T4_MAT24_kgmmms.key")]

for file in filtered_lsdyna:
    path_to_file = os.path.join(cluster_path, "toolchain", "files", "Lattice_Structures", file)
    shutil.copy(path_to_file, destination)

for file in filtered_lsdyna: # delete files from the origin
    path_to_file = os.path.join(cluster_path, "toolchain", "files", "Lattice_Structures", file)
    os.remove(path_to_file)
