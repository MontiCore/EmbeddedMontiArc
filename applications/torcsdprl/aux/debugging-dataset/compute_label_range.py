import h5py
import json
import numpy as np

f = open("debugging_dataset-conifg.json") 
config = json.loads(f.read())
debuggin_dataset_path = config["debugging_dataset_path"]
split = config["split"]
f.close()
train_h5 = h5py.File(debuggin_dataset_path+ "train.h5")

train_label = np.array(train_h5["predictions_label"])

range_lower = train_label.min(axis=0)    
range_upper = train_label.max(axis=0)

label_names = ["angle", "toMarking_L", "toMarking_M", "toMarking_R", "dist_L", \
    "dist_R", "toMarkingLL", "toMarkingML", "toMarkingMR", "toMarkingRR", \
    "dist_LL", "dist_MM", "dist_RR", "fast"]
print("------------------")
for name, l, u in zip(label_names, range_lower, range_upper):
    print(name + ": (" + str(np.round(l, decimals=2)) + ", " + str(np.round(u, decimals=2)) + ")")
print("------------------")


