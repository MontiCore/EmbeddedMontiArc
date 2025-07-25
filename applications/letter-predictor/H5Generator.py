import os
import sys

import h5py
import numpy as np
import idx2numpy
import matplotlib.pyplot as plt
import numpy as np
import cv2
from datetime import datetime
import gzip

train_images_file = None
train_labels_file = None

test_images_file = None
test_labels_file = None

targetDir = None
prefix = None

args = sys.argv
arg_i = 0
while arg_i < len(args):

    if args[arg_i] == "-t":
        targetDir = args[arg_i + 1] + "/"
    elif args[arg_i] == "-p":
        prefix = args[arg_i + 1]
    elif args[arg_i] == "-tri":
        train_images_file = args[arg_i + 1]
    elif args[arg_i] == "-trl":
        train_labels_file = args[arg_i + 1]
    elif args[arg_i] == "-tsi":
        test_images_file = args[arg_i + 1]
    elif args[arg_i] == "-tsl":
        test_labels_file = args[arg_i + 1]

    arg_i += 1



def reshapeImages(imageArr, path):
    saveArrayAsImage(imageArr, path)
    if path is None:
        path = ""

    i = 0
    newArray = []
    while i < len(imageArr):
        img = imageArr[i]
        img = img / 256
        imgpath = path + str(i) + ".png"
        img = np.expand_dims(img, axis=-1)
        t_img = np.transpose(img, (2, 0, 1)).astype(np.float32)
        newArray.append(t_img)
        print("Round: " + str(i) + " | Image: " + imgpath + " + Shape: " + (str(t_img.shape)))
        i += 1

    return newArray


def getTimeStamp():
    now = datetime.now()
    date_time = now.strftime("%Y-%m-%d_%H:%M:%S")
    return date_time


def getShape(arr):
    img = arr[0]
    length = len(arr)
    channels = img.shape[0]
    height = img.shape[1]
    width = img.shape[2]
    return length, channels, height, width


def checkTmpDir():
    try:
        os.remove(targetDir)
    except:
        print("Could not remove:" + targetDir)

    try:
        os.mkdir(targetDir)
        os.mkdir(targetDir + "train")
        os.mkdir(targetDir + "test")
    except:
        print("Could not create:" + targetDir)


def readImageFile(fileName: str, imgNumbers: int):
    f = gzip.open(fileName, 'r')

    image_size = 28
    num_images = imgNumbers

    f.read(16)
    buf = f.read(image_size * image_size * num_images)
    data = np.frombuffer(buf, dtype=np.uint8).astype(np.float32)
    data = data.reshape(num_images, 1, image_size, image_size)
    return data

def saveArrayAsImage(arr, targetDir: str):
    index = 0
    while index < len(arr):
        image = np.asarray(arr[index]).squeeze()
        cv2.imwrite(targetDir + str(index) + ".png", image)
        index += 1

if train_images_file is None or train_labels_file is None or test_images_file is None or test_labels_file is None:
    print("Dataset files not passed as argument. Aborting.")
    exit(1)

if targetDir is not None:
    checkTmpDir()

if prefix is None:
    prefix = getTimeStamp()

train_images_array = idx2numpy.convert_from_file(train_images_file)
train_images_array = reshapeImages(train_images_array, targetDir + "train/")

train_label_array = idx2numpy.convert_from_file(train_labels_file)
train_label_array = train_label_array.astype(np.float32)

train_file = h5py.File(prefix + "-train.h5", "w")
train_file.create_dataset("data", getShape(train_images_array), dtype=np.float32, data=train_images_array)
train_file.create_dataset("softmax_label", train_label_array.shape, dtype=np.float32, data=train_label_array)

train_file.close()



test_images_array = idx2numpy.convert_from_file(test_images_file)
test_images_array = reshapeImages(test_images_array, targetDir + "test/")

test_label_array = idx2numpy.convert_from_file(test_labels_file)
test_label_array = test_label_array.astype(np.float32)

test_file = h5py.File(prefix + "-test.h5", "w")
test_file.create_dataset("data", getShape(test_images_array), dtype=np.float32, data=test_images_array)
test_file.create_dataset("softmax_label", test_label_array.shape, dtype=np.float32, data=test_label_array)

test_file.close()
