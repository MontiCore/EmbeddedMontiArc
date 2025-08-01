import argparse
import os

import h5py
import numpy as np
import cv2

res = (256, 256)

parser = argparse.ArgumentParser(
    description='Convert pix2pix dataset to HDF5.')
parser.add_argument(
    '-p',
    '--path_directory',
    default='./datasets/facades',
    help='path to directory')
parser.add_argument(
    '-i',
    '--channels_in',
    default="3",
    help='images channels')
parser.add_argument(
    '-o',
    '--channels_out',
    default="3",
    help='target channels')
parser.add_argument(
    '-t',
    '--to_tensor',
    default='no',
    help='convert to tensor yes or no (default: no)'
)

def convert_images(img, channels, to_tensor=False):
    if channels == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    if channels == 1:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = np.expand_dims(img, axis=2)

    if to_tensor:
        img = img / 255.
        img = np.transpose(img, (2, 0, 1))

    return img

def get_image_paths(directory):
    image_paths = [os.path.join(directory, f) for f in os.listdir(directory) if (os.path.isfile(os.path.join(directory, f)) and ('.jpg' in f))]
    return image_paths

def get_dirs(directory):
    image_paths = [os.path.join(directory, f) for f in os.listdir(directory) if os.path.isdir(os.path.join(directory, f))]
    return image_paths

def get_images(path, channels_in, channels_out, to_tensor=False):
    """Get image data as uint8 array for given image.
    Parameters
    ----------
    path : str
        Path to image.
    Returns
    -------
    image_data : array of uint8
        Image byte string represented as array of uint8.
    image_target : array of uint8
        Image byte string represented as array of uint8.
    """
    # read and resize image
    image = cv2.imread(path, cv2.IMREAD_COLOR)
    image = cv2.resize(image, (res[0]*2,res[1]))

    # split input and target
    image_target, image_data = image[:, :res[1]], image[:, res[1]:]
    image_data = convert_images(image_data, channels_in, to_tensor=to_tensor)
    image_target = convert_images(image_target, channels_out, to_tensor=to_tensor)
    return image_data, image_target


def add_to_dataset(paths, images, targets, channels_in, channels_out, start=0, to_tensor=False):
    """Process all given ids and adds them to given datasets."""
    for i, path in enumerate(paths):
        image_data, image_target = get_images(path, channels_in, channels_out, to_tensor=to_tensor)
        images[start + i] = image_data
        targets[start + i] = image_target
    return i

def dir_to_dataset(path_to_dir, channels_in, channels_out, to_tensor):
    print("Dataset at: ", path_to_dir)
    print("channels data: ", channels_in)
    print("channels target: ", channels_out)

    # Create HDF5 dataset structure
    print('Creating HDF5 dataset structure.')
    parent_dir, target_dir = os.path.split(path_to_dir)
    fname = os.path.join(parent_dir, target_dir + '.h5')

    fh5 = h5py.File(fname, 'w')
    image_paths = get_image_paths(path_to_dir)

    # store class list for reference class ids as csv fixed-length numpy string
    # fh5.attrs['classes'] = np.string_(str.join(',', classes))
    if to_tensor:
        shape_in = (channels_in,) + res
        shape_out = (channels_out,) + res
    else:
        shape_in =  res + (channels_in,)
        shape_out = res + (channels_out,)

    # store images as variable length uint8 arrays
    train_images = fh5.create_dataset(
        'data', shape=(len(image_paths),)+shape_in)

    # store boxes as class_id, xmin, ymin, xmax, ymax
    train_target = fh5.create_dataset(
        'target_label', shape=(len(image_paths),)+shape_out)

    # process all ids and add to datasets
    print('Processing {}...'.format(path_to_dir))
    add_to_dataset(image_paths, train_images, train_target, channels_in, channels_in, to_tensor=to_tensor)

    print('Closing HDF5 file.')
    fh5.close()
    print('Done.')


def _main(args):
    path_directory = os.path.expanduser(args.path_directory)
    to_tensor = os.path.expanduser(args.to_tensor)
    if to_tensor == 'yes' or to_tensor == 'y':
        to_tensor = True
    elif to_tensor == 'no' or to_tensor == 'n':
        to_tensor = False
    else:
        print('Invalid argument, to tensor has to be yes or no (y or n)')

    channels_in = int(os.path.expanduser(args.channels_in))
    channels_out = int(os.path.expanduser(args.channels_out))

    for path_to_dir in get_dirs(path_directory):
        dir_to_dataset(path_to_dir, channels_in=channels_in, channels_out=channels_out, to_tensor=to_tensor)

def test_h5files(args):
    path_directory = os.path.expanduser(args.path_directory)

    to_tensor = os.path.expanduser(args.to_tensor)
    if to_tensor == 'yes' or to_tensor == 'y':
        to_tensor = True
    elif to_tensor == 'no' or to_tensor == 'n':
        to_tensor = False
    else:
        print('Invalid argument, to tensor has to be yes or no (y or n)')

    path_h5 = os.path.join(path_directory, 'train.h5')
    with h5py.File(path_h5, 'r') as fh5:
        print('Successfully opened.')
        print('Datasets: ')
        for key in fh5.keys():
            print(key)
            if to_tensor:
                data = fh5[key][0] * 255
                data = np.transpose(data, (1, 2, 0))
                data = cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
                cv2.imwrite('test_{}.png'.format(key), data)
            else:
                data = fh5[key][0]
                data = cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
                cv2.imwrite('test_{}.png'.format(key), data)

if __name__ == '__main__':
    _main(parser.parse_args())
    test_h5files(parser.parse_args())