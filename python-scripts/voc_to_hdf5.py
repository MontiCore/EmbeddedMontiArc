"""
https://github.com/allanzelener/YAD2K/blob/master/voc_conversion_scripts/voc_to_hdf5.py

Convert Pascal VOC 2007+2012 detection dataset to HDF5.
Does not preserve full XML annotations.
Combines all VOC subsets (train, val test) with VOC2012 train for full
training set as done in Faster R-CNN paper.
Code based on:
https://github.com/pjreddie/darknet/blob/master/scripts/voc_label.py
"""

import argparse
import os
import xml.etree.ElementTree as ElementTree

import h5py
import numpy as np

sets_from_2007 = [('2007', 'train'), ('2007', 'val')]
train_set = [('2012', 'train')]
val_set = [('2012', 'val')]
test_set = [('2007', 'test')]

classes = [
    "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat",
    "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person",
    "pottedplant", "sheep", "sofa", "train", "tvmonitor"
]

parser = argparse.ArgumentParser(
    description='Convert Pascal VOC 2007+2012 detection dataset to HDF5.')
parser.add_argument(
    '-p',
    '--path_to_voc',
    help='path to VOCdevkit directory',
    default='~/VOCdevkit')


def get_image_for_id(voc_path, year, image_id):
    """Get image data as uint8 array for given image.
    Parameters
    ----------
    voc_path : str
        Path to VOCdevkit directory.
    year : str
        Year of dataset containing image. Either '2007' or '2012'.
    image_id : str
        Pascal VOC identifier for given image.
    Returns
    -------
    image_data : array of uint8
        Compressed JPEG byte string represented as array of uint8.
    """
    fname = os.path.join(voc_path, 'VOC{}/JPEGImages/{}.jpg'.format(year,
                                                                    image_id))
    with open(fname, 'rb') as in_file:
        data = in_file.read()
    # Use of encoding based on: https://github.com/h5py/h5py/issues/745
    return np.fromstring(data, dtype='uint8')

def get_segmentation_for_id(voc_path, year, image_id):
    """Get image data as uint8 array for given image.
    Parameters
    ----------
    voc_path : str
        Path to VOCdevkit directory.
    year : str
        Year of dataset containing image. Either '2007' or '2012'.
    image_id : str
        Pascal VOC identifier for given image.
    Returns
    -------
    image_data : array of uint8
        Compressed JPEG byte string represented as array of uint8.
    """
    fname = os.path.join(voc_path, 'VOC{}/SegmentationClass/{}.png'.format(year,
                                                                    image_id))
    with open(fname, 'rb') as in_file:
        data = in_file.read()
    # Use of encoding based on: https://github.com/h5py/h5py/issues/745
    return np.fromstring(data, dtype='uint8')


def get_ids(voc_path, datasets):
    """Get image identifiers for corresponding list of dataset identifies.
    Parameters
    ----------
    voc_path : str
        Path to VOCdevkit directory.
    datasets : list of str tuples
        List of dataset identifiers in the form of (year, dataset) pairs.
    Returns
    -------
    ids : list of str
        List of all image identifiers for given datasets.
    """
    ids = []
    for year, image_set in datasets:
        id_file = os.path.join(voc_path, 'VOC{}/ImageSets/Segmentation/{}.txt'.format(
            year, image_set))
        with open(id_file, 'r') as image_ids:
            ids.extend(map(str.strip, image_ids.readlines()))
    return ids


def add_to_dataset(voc_path, year, ids, images, segmentations, start=0):
    """Process all given ids and adds them to given datasets."""
    for i, voc_id in enumerate(ids):
        image_data = get_image_for_id(voc_path, year, voc_id)
        seg_data = get_segmentation_for_id(voc_path, year, voc_id)
        images[start + i] = image_data
        segmentations[start + i] = seg_data
    return i


def _main(args):
    voc_path = os.path.expanduser(args.path_to_voc)
    train_ids = get_ids(voc_path, train_set)
    val_ids = get_ids(voc_path, val_set)
    test_ids = get_ids(voc_path, test_set)
    train_ids_2007 = get_ids(voc_path, sets_from_2007)
    total_train_ids = len(train_ids) + len(train_ids_2007)

    # Create HDF5 dataset structure
    print('Creating HDF5 dataset structure.')
    fname_train = os.path.join(voc_path, 'pascal_voc_07_12_train.hdf5')
    fname_val = os.path.join(voc_path, 'pascal_voc_07_12_val.hdf5')
    fname_test = os.path.join(voc_path, 'pascal_voc_07_12_test.hdf5')

    voc_h5file_train = h5py.File(fname_train, 'w')
    voc_h5file_val = h5py.File(fname_val, 'w')
    voc_h5file_test = h5py.File(fname_test, 'w')
    uint8_dt = h5py.special_dtype(
        vlen=np.dtype('uint8'))  # variable length uint8

    # store class list for reference class ids as csv fixed-length numpy string
    voc_h5file_train.attrs['classes'] = np.string_(str.join(',', classes))
    voc_h5file_val.attrs['classes'] = np.string_(str.join(',', classes))
    voc_h5file_test.attrs['classes'] = np.string_(str.join(',', classes))

    # store images as variable length uint8 arrays
    train_images = voc_h5file_train.create_dataset(
        'data', shape=(total_train_ids, ), dtype=uint8_dt)
    val_images = voc_h5file_val.create_dataset(
        'data', shape=(len(val_ids), ), dtype=uint8_dt)
    test_images = voc_h5file_test.create_dataset(
        'data', shape=(len(test_ids), ), dtype=uint8_dt)

    # store boxes as class_id, xmin, ymin, xmax, ymax
    train_segmentations = voc_h5file_train.create_dataset(
        'softmax_label', shape=(total_train_ids, ), dtype=uint8_dt)
    val_segmentations = voc_h5file_val.create_dataset(
        'softmax_label', shape=(len(val_ids), ), dtype=uint8_dt)
    test_segmentations = voc_h5file_test.create_dataset(
        'softmax_label', shape=(len(test_ids), ), dtype=uint8_dt)

    # process all ids and add to datasets
    print('Processing Pascal VOC 2007 datasets for training set.')
    last_2007 = add_to_dataset(voc_path, '2007', train_ids_2007, train_images,
                               train_segmentations)
    print('Processing Pascal VOC 2012 training set.')
    add_to_dataset(
        voc_path,
        '2012',
        train_ids,
        train_images,
        train_segmentations,
        start=last_2007 + 1)
    # print('Processing Pascal VOC 2012 val set.')
    add_to_dataset(voc_path, '2012', val_ids, val_images, val_segmentations)
    print('Processing Pascal VOC 2007 test set.')
    add_to_dataset(voc_path, '2007', test_ids, test_images, test_segmentations)

    print('Closing HDF5 file.')
    voc_h5file_train.close()
    voc_h5file_val.close()
    voc_h5file_test.close()
    print('Done.')


if __name__ == '__main__':
    _main(parser.parse_args())