import json 
import h5py

f = open("debugging_dataset-conifg.json") 
config = json.loads(f.read())
debuggin_dataset_path = config["debugging_dataset_path"]
dataset_path = config["dataset_path"]
f.close()

test_h5 = h5py.File(dataset_path)

for key in test_h5:
    print(key)

print(test_h5["data"].shape)
print(test_h5["predictions_label"].shape)
