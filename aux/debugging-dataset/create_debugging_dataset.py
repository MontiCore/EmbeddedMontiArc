import h5py
import json
import numpy as np

f = open("debugging_dataset-conifg.json") 
config = json.loads(f.read())
dataset_path = config["dataset_path"]
debuggin_dataset_path = config["debugging_dataset_path"]
debugging_dataset_size = config["debugging_dataset_size"]
split = config["split"]
f.close()

data_file = h5py.File(dataset_path, "r")
debugging_data_file = h5py.File(debuggin_dataset_path + "/train.h5", "w")
debugging_test_file = h5py.File(debuggin_dataset_path + "/test.h5", "w")

# take random samples of the data set
samples = np.random.choice(40000, debugging_dataset_size, replace=False)
data_samples = samples[ :int(split * debugging_dataset_size)]
test_samples = samples[int(split * debugging_dataset_size): ]
data_samples.sort()
test_samples.sort()

print("Creating debugging dataset. This may take a minute.")
debugging_data_file["data"] = data_file["data"][data_samples, :, :, :]
debugging_data_file["predictions_label"] = data_file["predictions_label"][data_samples, :]
debugging_test_file["data"] = data_file["data"][test_samples, :, :, :]
debugging_test_file["predictions_label"] = data_file["predictions_label"][test_samples, :]

