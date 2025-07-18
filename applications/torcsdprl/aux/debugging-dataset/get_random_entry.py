import h5py
import json
import numpy as np
from PIL import Image

def print_label(label_value, label_range):
    label_names = ["angle", "toMarking_L", "toMarking_M", "toMarking_R", "dist_L", \
        "dist_R", "toMarkingLL", "toMarkingML", "toMarkingMR", "toMarkingRR", \
        "dist_LL", "dist_MM", "dist_RR", "fast"]
    label_units = ["rad", "m", "m", "m", "m", "m", "m", "m", "m", "m", "m", "m", "m", ""]
    denormalized_label_values = denormalize(label_value, label_range)
    
    print("------------------")
    print("always-on affordance")
    print("%s : %.3f \t (%.2f rad = %.2f deg)" % (label_names[0], label_value[0], \
        denormalized_label_values[0], denormalized_label_values[0] / np.pi * 180))
    print(label_names[-1] + ": " + str(label_value[-1]))
    print("------------------")

    print("on-mark affordance:")
    for name, value, denormalized_value in zip(label_names[1:6], label_value[1:6], denormalized_label_values[1:6]):
        print("%s : %.2f \t (%.2f m)" % (name, value, denormalized_value))
        #print(name + ": " + str(value))
    distL = label_value[1]
    distR = label_value[3]
    if(distL > 0 and distR < 1):
        print("track width: %.2f" % (denormalized_label_values[3] - denormalized_label_values[1]))
    print("------------------")

    print("in-lane affordance:")
    for name, value, denormalized_value in zip(label_names[6: -1], label_value[6: -1], denormalized_label_values[6: -1]):
        print("%s : %.2f \t (%.2f m)" % (name, value, denormalized_value))

    distLL = label_value[6]
    distRR = label_value[9]
    angle = denormalized_label_values[0]
    speed = 1

    if (distLL > 0 and distRR < 1):
        track_width = denormalized_label_values[9] - denormalized_label_values[6]
        track_pos = (track_width - denormalized_label_values[9]) / track_width
        score = (speed * np.cos(angle)) - (speed * np.sin(angle)) - (speed * track_pos)
        print("\ntrack width: %.2f" % (track_width))
        print("track pos: %.2f" %(track_pos))
        print("reward: %.2f" %(score))
    elif(distLL > 0 and distRR == 1):
        track_width = denormalized_label_values[8] - denormalized_label_values[6]
        track_pos = (track_width - denormalized_label_values[8]) / track_width
        score = (speed * np.cos(angle)) - (speed * np.sin(angle)) - (speed * track_pos)
        print("\ntrack width: %.2f" % (track_width))
        print("track pos: %.2f" %(track_pos))
        print("reward: %.2f" %(score))
    elif(distLL == 0 and distRR < 1):
        track_width = denormalized_label_values[9] - denormalized_label_values[7]
        track_pos = (track_width - denormalized_label_values[9]) / track_width
        score = (speed * np.cos(angle)) - (speed * np.sin(angle)) - (speed * track_pos)
        print("\ntrack width: %.2f" % (track_width))
        print("track pos: %.2f" %(track_pos))
        print("reward: %.2f" %(score))
    print("------------------")


def denormalize(label, label_range):
    return np.array([denormalize_value(value, label_range[i]) for i, value in enumerate(label)])


def denormalize_value(value, label_range):
    return value * (label_range[1] - label_range[0]) + label_range[0]


f = open("debugging_dataset-conifg.json") 
config = json.loads(f.read())
debuggin_dataset_path = config["debugging_dataset_path"]
debuggin_dataset_size = config["debugging_dataset_size"]
mxnet_image_format = config["mxnet_image_format"]
split = config["split"]
show_test_image = config["show_test_image"]
f.close()

f = open("../preprocessing/preprocessing_config.json")
config = json.loads(f.read())
ANGLE_RANGE = config["angle_range"]
TOMARKING_L_RANGE = config["toMarking_L_range"]
TOMARKING_M_RANGE = config["toMarking_M_range"]
TOMARKING_R_RANGE = config["toMarking_R_range"]
DIST_L_RANGE = config["dist_L_range"]
DIST_R_RANGE = config["dist_R_range"]
TOMARKING_LL_RANGE = config["toMarking_LL_range"]
TOMARKING_ML_RANGE = config["toMarking_ML_range"]
TOMARKING_MR_RANGE = config["toMarking_MR_range"]
TOMARKING_RR_RANGE = config["toMarking_RR_range"]
DIST_LL_RANGE = config["dist_LL_range"]
DIST_MM_RANGE = config["dist_MM_range"]
DIST_RR_RANGE = config["dist_RR_range"]
FAST_RANGE = config["fast_range"]
f.close()

LABEL_RANGE = [ANGLE_RANGE, TOMARKING_L_RANGE, TOMARKING_M_RANGE, TOMARKING_R_RANGE, \
DIST_L_RANGE, DIST_R_RANGE, TOMARKING_LL_RANGE, TOMARKING_ML_RANGE, TOMARKING_MR_RANGE, \
TOMARKING_RR_RANGE, DIST_LL_RANGE, DIST_MM_RANGE, DIST_RR_RANGE, FAST_RANGE]

debugging_train_file = h5py.File(debuggin_dataset_path + "/train.h5", "r")
debugging_test_file = h5py.File(debuggin_dataset_path + "/train.h5", "r")


train_entry = np.random.randint(debuggin_dataset_size * split)
test_entry = np.random.randint(debuggin_dataset_size * (1 - split))
print("TRAIN LABEL")
print_label(debugging_train_file["predictions_label"][train_entry], LABEL_RANGE)
if show_test_image:
    print("\nTEST LABEL")
    print_label(debugging_train_file["predictions_label"][test_entry], LABEL_RANGE)

if mxnet_image_format:
    img = Image.fromarray(np.moveaxis(debugging_train_file["data"][train_entry], [0], [2]), 'RGB')
else:
    img = Image.fromarray(debugging_train_file["data"][train_entry], 'RGB')

if mxnet_image_format and show_test_image:
    img2 = Image.fromarray(np.moveaxis(debugging_test_file["data"][test_entry], [0], [2]), 'RGB')
elif show_test_image:
    img2 = Image.fromarray(debugging_test_file["data"][test_entry], 'RGB')
img.show()
if show_test_image:
    img2.show()
