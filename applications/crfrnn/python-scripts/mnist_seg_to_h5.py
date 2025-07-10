import numpy as np
import mxnet as mx
import h5py
import os
import argparse

classes = [str(i) for i in range(10)]

parser = argparse.ArgumentParser(
    description='Convert mnist dataset to HDF5.')
parser.add_argument(
    '-p',
    '--path_to_mnist',
    help='path to mnist directory',
    default='/home/treiber/.mxnet/datasets/mnist')

def labels_as_segmented(data, label, thresh=0):
    ### TODO thresh to control pixels
    res = []
    for d, l in zip(data, label):
        copy = d.copy()
        if l == 0:
            l = 10
        copy[d > thresh] = l
        res.append(copy)
    return np.array(res).reshape(-1,1,28,28)

def test_convert(train_data):
    import matplotlib.pyplot as plt

    res = np.hstack([el[0] for el in train_data[20:30]])
    print(res.shape)
    print(type(res[0,0]))
    plt.imshow(res), plt.show()

def _main(args):
    mx.random.seed(42)
    mnist = mx.test_utils.get_mnist()
    mnist_path = args.path_to_mnist

    # Create HDF5 dataset structure
    print('Creating HDF5 dataset structure.')
    fname_train = os.path.join(mnist_path, 'mnist_train.hdf5')
    fname_test = os.path.join(mnist_path, 'mnist_test.hdf5')

    mnist_h5file_train = h5py.File(fname_train, 'w')
    mnist_h5file_test = h5py.File(fname_test, 'w')

    # store class list for reference class ids as csv fixed-length numpy string
    mnist_h5file_train.attrs['classes'] = np.string_(str.join(',', classes))
    mnist_h5file_test.attrs['classes'] = np.string_(str.join(',', classes))

    train_data = (mnist['train_data'] * 255).astype(np.uint8)
    test_data = (mnist['test_data'] * 255).astype(np.uint8)

    test_convert(train_data)

    ### TODO something something can't convert
    # store images as variable length uint8 arrays
    train_images = mnist_h5file_train.create_dataset(
        'data', data=train_data, dtype=np.uint8)
    test_images = mnist_h5file_test.create_dataset(
        'data', data=test_data, dtype=np.uint8)

    train_label = labels_as_segmented(mnist['train_data'], mnist['train_label'])
    test_label = labels_as_segmented(mnist['test_data'], mnist['test_label'])

    # test_convert(test_label)

    # store boxes as class_id, xmin, ymin, xmax, ymax
    train_segmentations = mnist_h5file_train.create_dataset(
        'softmax_label', data=train_label, dtype=np.uint8)
    test_segmentations = mnist_h5file_test.create_dataset(
        'softmax_label', data=test_label, dtype=np.uint8)

    print('Closing HDF5 file.')
    mnist_h5file_train.close()
    mnist_h5file_test.close()
    print('Done.')

def testtest(args):
    print(args.path_to_mnist)
    # Fixing the random seed
    mx.random.seed(42)

    mnist = mx.test_utils.get_mnist()

    # keys = ['test_data', 'test_label', 'train_data', 'train_label']

    for key in mnist:
        print('key: ', key)
        print('shape: ', mnist[key].shape)

    print('classes: ', classes)


if __name__ == '__main__':
    _main(parser.parse_args())
    # testtest(parser.parse_args())