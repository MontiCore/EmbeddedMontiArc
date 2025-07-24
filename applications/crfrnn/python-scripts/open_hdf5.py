import io
import os

import cv2
import h5py
import matplotlib.pyplot as plt
import mxnet as mx
import mxnet.gluon
from mxnet.gluon.data import DataLoader, ArrayDataset
from mxnet.gluon.data.vision import transforms
import numpy as np
import PIL

_input_shape = (480, 480, 3)
_mean_voc = [.485, .456, .406]
_std_voc = [.229, .224, .225]
_ctx = mx.cpu(0)

def get_filehandle(path='/home/treiber/.mxnet/datasets/voc/pascal_voc_07_12.hdf5'):
    return h5py.File(path, 'r')

def read_image_PIL(filehandle, num, split='train'):
    image = PIL.Image.open(io.BytesIO(filehandle[split+'/images'][num]))
    return image

def read_image_cv2(filehandle, num, split='train'):
    image_stream = io.BytesIO(filehandle[split+'/images'][num])
    file_bytes = np.asarray(bytearray(image_stream.read()), dtype=np.uint8)
    image = cv2.imdecode(file_bytes, cv2.IMREAD_COLOR)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    return image

def read_label_cv2(filehandle, num, split='train'):
    label_stream = io.BytesIO(filehandle[split+'/labels'][num])
    file_bytes = np.asarray(bytearray(label_stream.read()), dtype=np.uint8)
    label = cv2.imdecode(file_bytes, cv2.IMREAD_COLOR)
    label = cv2.cvtColor(label, cv2.COLOR_BGR2RGB)
    return label

def get_mxnet_nd_array(input):
    return 42

def get_data(filehandle, dataset='images', split='train'):
    data = filehandle[split+'/'+dataset]
    images = []
    ### TODO remove cap
    for el in data[:10]:
        stream = io.BytesIO(el)
        byte_image = np.asarray(bytearray(stream.read()), dtype=np.uint8)
        image = np.array(cv2.imdecode(byte_image, cv2.IMREAD_COLOR))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = cv2.resize(image, (_input_shape[0], _input_shape[1]))
        images.append(image)

    return mx.nd.array(np.array(images), ctx=_ctx)

def get_voc_input_transform():
    return transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize([.485, .456, .406], [.229, .224, .225]),
    ])

def get_dataloader(dataset, batch_size=32):
    # transform dataset
    transformer = get_voc_input_transform()
    dataset = dataset.transform_first(transformer)

    loader = DataLoader(
        dataset, batch_size, shuffle=True, last_batch='rollover',
        num_workers=batch_size)
    return loader

def test_single_image():
    num = 29
    filehandle = get_filehandle(path='/home/treiber/.mxnet/datasets/voc/pascal_voc_07_12.hdf5')
    image = read_image_cv2(filehandle, num)
    label = read_label_cv2(filehandle, num)
    plt.imshow(np.hstack([image, label]))
    plt.show()

def test_all_images():
    filehandle = get_filehandle(path='/home/treiber/.mxnet/datasets/voc/pascal_voc_07_12.hdf5')
    images = get_data(filehandle, dataset='images', split='train')
    labels = get_data(filehandle, dataset='labels', split='train')
    # print(images.shape)
    dataset = ArrayDataset(images, labels)
    loader = get_dataloader(dataset, batch_size=10)
    for image in loader:
        print(image)

if __name__ == '__main__':
    test_all_images()
