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

### not necessary, pix2pix states that segmentation images are provided as RGB for training
def segment_one_hot_encode(img, cmap):
    img = np.array(img)
    h, w = img.shape[:2]
    palette = [cmap[n:n+3] for n in range(0, len(cmap), 3)]
    out = np.zeros((len(cmap), h, w))

    for n in range(h):
        for m in range(w):
            for i, color in enumerate(cmap):
                if img[n,m] == color:
                    out[i,n,m] = 1

def convert_images(img, channels, to_tensor=False):
    if channels == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    if channels == 1:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = np.expand_dims(img, axis=2)

    img = img / 255.
    if to_tensor:
        img = np.transpose(img, (2, 0, 1))

    return img

def get_paths(directory):
    image_paths = [os.path.join(directory, f) for f in os.listdir(directory) if (os.path.isfile(os.path.join(directory, f)) and ('.jpg' in f))]
    return image_paths

def normalize_img(img, mean, std):
    img = img / 255.
    img = (img - mean) / std
    return img

def denormalize_img(img, mean, std):
    img = np.array(((img * std) + mean)*255, dtype=np.uint8)
    return img

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
    image_data, image_target = image[:, :res[1]], image[:, res[1]:]
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


def _main(args):
    path_directory = os.path.expanduser(args.path_directory)
    to_tensor = os.path.expanduser(args.to_tensor)
    if to_tensor == 'yes' or to_tensor == 'y':
        to_tensor = True
    elif to_tensor == 'no' or to_tensor == 'n':
        to_tensor = False
    else:
        print('Invalid argument, to tensor has to be yes or no (y or n)')

    paths_train = get_paths(os.path.join(path_directory,'train'))
    paths_test = get_paths(os.path.join(path_directory,'test'))

    channels_in = int(os.path.expanduser(args.channels_in))
    channels_out = int(os.path.expanduser(args.channels_out))

    print("Dataset at: ", path_directory)
    print("channels data: ", channels_in)
    print("channels target: ", channels_out)

    # Create HDF5 dataset structure
    print('Creating HDF5 dataset structure.')
    fname_train = os.path.join(path_directory, 'train.h5')
    fname_test = os.path.join(path_directory, 'test.h5')

    fh5_train = h5py.File(fname_train, 'w')
    fh5_test = h5py.File(fname_test, 'w')

    # store class list for reference class ids as csv fixed-length numpy string
    # fh5_train.attrs['classes'] = np.string_(str.join(',', classes))

    if to_tensor:
        shape_in = (channels_in,) + res
        shape_out = (channels_out,) + res
    else:
        shape_in =  res + (channels_in,)
        shape_out = res + (channels_out,)

    # store images as variable length uint8 arrays
    train_images = fh5_train.create_dataset(
        'data', shape=(len(paths_train),)+shape_in)
    test_images = fh5_test.create_dataset(
        'data', shape=(len(paths_test),)+shape_in)

    # store boxes as class_id, xmin, ymin, xmax, ymax
    train_target = fh5_train.create_dataset(
        'target', shape=(len(paths_train),)+shape_out)
    test_target = fh5_test.create_dataset(
        'target', shape=(len(paths_test),)+shape_out)

    # process all ids and add to datasets
    print('Processing...')
    add_to_dataset(paths_train, train_images, train_target, channels_in, channels_in, to_tensor=to_tensor)
    add_to_dataset(paths_test, test_images, test_target, channels_in, channels_out, to_tensor=to_tensor)

    print('Closing HDF5 file.')
    fh5_train.close()
    fh5_test.close()
    print('Done.')

def test_result():
    # with h5py.File("/home/treiber/.mxnet/datasets/voc/train.h5", 'r') as train:
    with h5py.File(os.path.expanduser("~/.mxnet/datasets/voc/train.h5"), 'r') as train:
        # print(train.keys())
        train_img = np.array(train["data"][0])
        train_img = np.transpose(train_img, (1, 2, 0))
        train_img = denormalize_img(train_img, voc_mean, voc_std)
        print(train_img.shape)
        train_img = cv2.cvtColor(train_img, cv2.COLOR_RGB2BGR)
        cv2.imshow('train_img', train_img)
        cv2.waitKey(0)
        label = np.reshape(train["softmax_label"][0][0], (480, 480))
        cv2.imshow('label', label)
        cv2.waitKey(0)

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
            test_data = fh5['data'][0] * 255
            test_data = np.transpose(test_data, (1, 2, 0))
            test_data = cv2.cvtColor(test_data, cv2.COLOR_RGB2BGR)
            cv2.imwrite('test_data.png', test_data)
            test_target = fh5['target'][0] * 255
            test_target = np.transpose(test_target, (1, 2, 0))
            test_target = cv2.cvtColor(test_target, cv2.COLOR_RGB2BGR)
            cv2.imwrite('test_target.png', test_target)
        else:
            test_data = fh5['data'][0] * 255
            test_data = cv2.cvtColor(test_data, cv2.COLOR_RGB2BGR)
            cv2.imwrite('test_data.png', test_data)
            test_target = fh5['target'][0] * 255
            test_target = cv2.cvtColor(test_target, cv2.COLOR_RGB2BGR)
            cv2.imwrite('test_target.png', test_target)



if __name__ == '__main__':
    _main(parser.parse_args())
    # test_result()
    # test_normalize()
    # test_mean()
    # paths = get_paths('/home/jt529748/git/crfrnn/python-scripts/pix2pix_datasets/datasets/facades/train')
    # cmap = get_cmap(paths)
    # print(cmap)
    # test_h5files(parser.parse_args())