import configparser
import glob
import os
import shutil

config = configparser.ConfigParser() # Reading config
config.read('../../../../../config.ini')
config = config['DEFAULT']
cluster_path = os.path.join("home", config['ClusterUser'], config['ClusterWorkingDirectory'])

print("Clean-up...")
old_lsdyna = glob.glob(os.path.join(cluster_path, "src", "main", "cluster", "toolchain", "preprocessing", "raw", "run__01", "*"))
for file in old_lsdyna:
    path_to_file = os.path.join(cluster_path, "src", "main", "cluster", "toolchain", "preprocessing", "raw", "run__01", file)
    os.remove(path_to_file)
os.remove(os.path.join(cluster_path, "src", "main", "cluster", "toolchain", "preprocessing", "raw", "Input.csv"))
temp_path = os.path.join(cluster_path, "src", "main", "cluster", "toolchain", "preprocessing", "lsdynabin_to_train", "__pycache__")
shutil.rmtree(temp_path)
