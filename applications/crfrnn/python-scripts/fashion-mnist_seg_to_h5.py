import numpy as np
import mxnet as mx
import h5py
import os
import argparse


"""
https://beta.mxnet.io/api/gluon/_autogen/mxnet.gluon.data.vision.datasets.FashionMNIST.html
"""

classes = [str(i) for i in range(10)]

parser = argparse.ArgumentParser(
    description='Convert fashionmnist dataset to HDF5.')
parser.add_argument(
    '-p',
    '--path_to_fashionmnist',
    help='path to fashionmnist directory',
    default='/home/treiber/.mxnet/datasets/fashionmnist')

def labels_as_segmented(data, label, thresh=0):
    res = []
    for d, l in zip(data, label):
        copy = d.copy()
        if l == 0:
            l = 10
        copy[d > thresh] = l
        res.append(copy)
    res = np.array(res).reshape(-1,1,28,28)
    print('res shape: ', res.shape)
    return res

def test_convert(train_data):
    import matplotlib.pyplot as plt

    res = np.hstack([el[0] for el in train_data[20:30]])
    print(res.shape)
    print(type(res[0,0]))
    plt.imshow(res), plt.show()

def _main(args):
    mx.random.seed(42)
    fashionmnist = get_fashionmnist_asdict(args)
    fashionmnist_path = args.path_to_fashionmnist

    # Create HDF5 dataset structure
    print('Creating HDF5 dataset structure.')
    fname_train = os.path.join(fashionmnist_path, 'fashionmnist_train.hdf5')
    fname_test = os.path.join(fashionmnist_path, 'fashionmnist_test.hdf5')

    fashionmnist_h5file_train = h5py.File(fname_train, 'w')
    fashionmnist_h5file_test = h5py.File(fname_test, 'w')

    # store class list for reference class ids as csv fixed-length numpy string
    fashionmnist_h5file_train.attrs['classes'] = np.string_(str.join(',', classes))
    fashionmnist_h5file_test.attrs['classes'] = np.string_(str.join(',', classes))

    train_data = (fashionmnist['train_data'] * 255).astype(np.uint8)
    test_data = (fashionmnist['test_data'] * 255).astype(np.uint8)

    test_convert(train_data)

    # store images as variable length uint8 arrays
    train_images = fashionmnist_h5file_train.create_dataset(
        'data', data=train_data, dtype=np.uint8)
    test_images = fashionmnist_h5file_test.create_dataset(
        'data', data=test_data, dtype=np.uint8)

    train_label = labels_as_segmented(fashionmnist['train_data'], fashionmnist['train_label'])
    test_label = labels_as_segmented(fashionmnist['test_data'], fashionmnist['test_label'])

    print('train_label shape: ', train_label.shape)

    # test_convert(test_label)

    # store boxes as class_id, xmin, ymin, xmax, ymax
    train_segmentations = fashionmnist_h5file_train.create_dataset(
        'softmax_label', data=train_label, dtype=np.uint8)
    test_segmentations = fashionmnist_h5file_test.create_dataset(
        'softmax_label', data=test_label, dtype=np.uint8)

    print('Closing HDF5 file.')
    fashionmnist_h5file_train.close()
    fashionmnist_h5file_test.close()
    print('Done.')

def get_fashionmnist_asdict(args):
    fashionmnistdict = {}
    print(args.path_to_fashionmnist)

    fashionmnist_train = mx.gluon.data.vision.FashionMNIST(root=args.path_to_fashionmnist, train=True)

    train_images = []
    train_labels = []
    for image, label in fashionmnist_train:
        train_images.append(image.asnumpy())
        train_labels.append(label)

    print(train_images[-1].shape)
    fashionmnistdict['train_data'] = np.concatenate(train_images).reshape((60000,1,28,28))
    fashionmnistdict['train_label'] = np.array(train_labels)

    # import matplotlib.pyplot as plt
    # plt.imshow(fashionmnistdict['train_data'][-1][0]), plt.show()

    fashionmnist_test = mx.gluon.data.vision.FashionMNIST(root=args.path_to_fashionmnist, train=False)

    test_images = []
    test_labels = []
    for image, label in fashionmnist_test:
        test_images.append(image.asnumpy())
        test_labels.append(label)

    print(test_images[-1].shape)
    fashionmnistdict['test_data'] = np.concatenate(test_images).reshape((10000,1,28,28))
    fashionmnistdict['test_label'] = np.array(test_labels)

    for key in fashionmnistdict:
        print('key: ', key)
        print('shape: ', fashionmnistdict[key].shape)

    return fashionmnistdict

def testtest(args):
    print(args.path_to_fashionmnist)
    # Fixing the random seed
    mx.random.seed(42)

    fashionmnist = mx.gluon.data.vision.FashionMNIST(root=args.path_to_fashionmnist, train=True)


    # keys = ['test_data', 'test_label', 'train_data', 'train_label']

    # print(fashionmnist)

    for image, label in fashionmnist:
        print('image: ', image.asnumpy()[:,:,0].shape)
        print('label: ', label)

    # print('classes: ', classes)


if __name__ == '__main__':
    _main(parser.parse_args())
    # testtest(parser.parse_args())
    # get_fashionmnist_asdict(pasrser.parse_args())