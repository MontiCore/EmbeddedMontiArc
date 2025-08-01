from PIL.Image import new
import h5py
import json
import numpy as np
from tqdm import tqdm
import cv2
from PIL import Image


f = open("debugging_dataset-conifg.json") 
config = json.loads(f.read())
dataset_path = config["debugging_dataset_path"]
small_img_dataset_path = config["small_img_dataset_path"]
f.close()

train_data_file = h5py.File(dataset_path + "train.h5", "r")
small_img_train = h5py.File(small_img_dataset_path + "/train.h5", "w")

train_size = train_data_file["data"].shape[0]
train_data = small_img_train.create_dataset("data", (train_size, 3, 64, 64) , dtype='uint8', chunks=True) 
train_lbl = small_img_train.create_dataset("predictions_label", (train_size, 14), dtype='f', chunks=True)

test_data_file = h5py.File(dataset_path + "test.h5", "r")
small_img_test = h5py.File(small_img_dataset_path + "/test.h5", "w")

test_size = test_data_file["data"].shape[0]
test_data = small_img_test.create_dataset("data", (test_size, 3, 64, 64) , dtype='uint8', chunks=True) 
test_lbl = small_img_test.create_dataset("predictions_label", (test_size, 14), dtype='f', chunks=True)


# slize_size limits the memory cosumption, by only loading slizes of size <500MB into memory
train_slize_size = min(train_size - 1, int(500e3 / (train_data[0].size * train_data[0].itemsize)))
test_slize_size = min(test_size - 1, int(500e3 / (test_data[0].size * test_data[0].itemsize)))

print("\ndownsample images in training-set")
with tqdm(total=int(train_size / train_slize_size)) as bar:
    for i in range(int(train_size / train_slize_size)):
        tmp = train_data_file["data"][i * train_slize_size: (i + 1) * train_slize_size]
        for j, k in enumerate(range(i * train_slize_size, (i + 1) * train_slize_size)): # range obvs
            img = tmp[j]
            new_img = cv2.resize(np.moveaxis(img, [0], [2]), (64, 64), interpolation = cv2.INTER_CUBIC)
            train_data[k] = np.moveaxis(new_img, [2], [0])
        train_lbl[i * train_slize_size: (i + 1) * train_slize_size] = train_data_file["predictions_label"][i * train_slize_size: (i + 1) * train_slize_size]
        bar.update(1)

print("\ndownsample images in test-set")
with tqdm(total=int(test_size / test_slize_size)) as bar:
    for i in range(int(test_size / test_slize_size)):
        tmp = test_data_file["data"][i * test_slize_size: (i + 1) * test_slize_size]
        for j, k in enumerate(range(i * test_slize_size, (i + 1) * test_slize_size)):
            img = tmp[j]
            new_img = cv2.resize(np.moveaxis(img, [0], [2]), (64, 64), interpolation = cv2.INTER_CUBIC)
            test_data[k] = np.moveaxis(new_img, [2], [0])
        test_lbl[i * test_slize_size: (i + 1) * test_slize_size] = test_data_file["predictions_label"][i * test_slize_size: (i + 1) * test_slize_size]
        bar.update(1)



n = int(np.random.randint(train_size - 1))
print(small_img_train["predictions_label"][n])
img = Image.fromarray(np.moveaxis(small_img_train["data"][n], [0], [2]), 'RGB')
img.show()