import os
import h5py
import mxnet as mx
import logging
import sys
import numpy as np
import cv2
import importlib
from mxnet import nd
from mxnet.gluon.data.vision import transforms

class CNNDataLoader_cNNSegment_connector_predictor:
    _input_names_ = ['data']
    _output_names_ = ['softmax_label']

    def __init__(self):
        self._data_dir = "../resources/fashionmnist_2x2/"

    def load_data(self, batch_size, shuffle=False):
        train_h5, test_h5 = self.load_h5_files()

        train_data = {}
        data_mean = {}
        data_std = {}
        train_images = {}

        for input_name in self._input_names_:
            train_data[input_name] = train_h5[input_name]
            data_mean[input_name + '_'] = nd.array(train_h5[input_name][:].mean(axis=0))
            data_std[input_name + '_'] = nd.array(train_h5[input_name][:].std(axis=0) + 1e-5)

            if 'images' in train_h5:
                train_images = train_h5['images']

        train_label = {}
        index = 0
        for output_name in self._output_names_:
            train_label[index] = train_h5[output_name]
            index += 1

        train_iter = mx.io.NDArrayIter(data=train_data,
                                       label=train_label,
                                       batch_size=batch_size,
                                       shuffle=shuffle)

        test_iter = None

        if test_h5 != None:
            test_data = {}
            test_images = {}
            for input_name in self._input_names_:
                test_data[input_name] = test_h5[input_name]

                if 'images' in test_h5:
                    test_images = test_h5['images']

            test_label = {}
            index = 0
            for output_name in self._output_names_:
                test_label[index] = test_h5[output_name]
                index += 1

            test_iter = mx.io.NDArrayIter(data=test_data,
                                          label=test_label,
                                          batch_size=batch_size)

        return train_iter, test_iter, data_mean, data_std, train_images, test_images

    def load_preprocessed_data(self, batch_size, preproc_lib, shuffle=False):
        train_h5, test_h5 = self.load_h5_files()

        wrapper = importlib.import_module(preproc_lib)
        instance = getattr(wrapper, preproc_lib)()
        instance.init()
        lib_head, _sep, tail = preproc_lib.rpartition('_')
        inp = getattr(wrapper, lib_head + "_input")()

        train_data = {}
        train_label = {}
        data_mean = {}
        data_std = {}

        shape_output = self.preprocess_data(instance, inp, 0, train_h5)
        train_len = len(train_h5[self._input_names_[0]])

        for input_name in self._input_names_:
            if type(getattr(shape_output, input_name + "_out")) == np.ndarray:
                cur_shape = (train_len,) + getattr(shape_output, input_name + "_out").shape
            else:
                cur_shape = (train_len, 1)
            train_data[input_name] = mx.nd.zeros(cur_shape)
        for output_name in self._output_names_:
            if type(getattr(shape_output, output_name + "_out")) == nd.array:
                cur_shape = (train_len,) + getattr(shape_output, output_name + "_out").shape
            else:
                cur_shape = (train_len, 1)
            train_label[output_name] = mx.nd.zeros(cur_shape)

        for i in range(train_len):
            output = self.preprocess_data(instance, inp, i, train_h5)
            for input_name in self._input_names_:
                train_data[input_name][i] = getattr(output, input_name + "_out")
            for output_name in self._output_names_:
                train_label[output_name][i] = getattr(shape_output, output_name + "_out")

        for input_name in self._input_names_:
            data_mean[input_name + '_'] = nd.array(train_data[input_name][:].mean(axis=0))
            data_std[input_name + '_'] = nd.array(train_data[input_name][:].asnumpy().std(axis=0) + 1e-5)

        if 'images' in train_h5:
            train_images = train_h5['images']

        train_iter = mx.io.NDArrayIter(data=train_data,
                                       label=train_label,
                                       batch_size=batch_size,
                                       shuffle=shuffle)

        test_data = {}
        test_label = {}

        shape_output = self.preprocess_data(instance, inp, 0, test_h5)
        test_len = len(test_h5[self._input_names_[0]])

        for input_name in self._input_names_:
            if type(getattr(shape_output, input_name + "_out")) == np.ndarray:
                cur_shape = (test_len,) + getattr(shape_output, input_name + "_out").shape
            else:
                cur_shape = (test_len, 1)
            test_data[input_name] = mx.nd.zeros(cur_shape)
        for output_name in self._output_names_:
            if type(getattr(shape_output, output_name + "_out")) == nd.array:
                cur_shape = (test_len,) + getattr(shape_output, output_name + "_out").shape
            else:
                cur_shape = (test_len, 1)
            test_label[output_name] = mx.nd.zeros(cur_shape)

        for i in range(test_len):
            output = self.preprocess_data(instance, inp, i, test_h5)
            for input_name in self._input_names_:
                test_data[input_name][i] = getattr(output, input_name + "_out")
            for output_name in self._output_names_:
                test_label[output_name][i] = getattr(shape_output, output_name + "_out")

        if 'images' in test_h5:
            test_images = test_h5['images']

        test_iter = mx.io.NDArrayIter(data=test_data,
                                       label=test_label,
                                       batch_size=batch_size)

        return train_iter, test_iter, data_mean, data_std, train_images, test_images

    def preprocess_data(self, instance_wrapper, input_wrapper, index, data_h5):
        for input_name in self._input_names_:
            data = data_h5[input_name][0]
            attr = getattr(input_wrapper, input_name)
            if (type(data)) == np.ndarray:
                data = np.asfortranarray(data).astype(attr.dtype)
            else:
                data = type(attr)(data)
            setattr(input_wrapper, input_name, data)
        for output_name in self._output_names_:
            data = data_h5[output_name][0]
            attr = getattr(input_wrapper, output_name)
            if (type(data)) == np.ndarray:
                data = np.asfortranarray(data).astype(attr.dtype)
            else:
                data = type(attr)(data)
            setattr(input_wrapper, output_name, data)
        return instance_wrapper.execute(input_wrapper)

    def load_h5_files(self):
        train_h5 = None
        test_h5 = None
        train_path = self._data_dir + "train.h5"
        test_path = self._data_dir + "test.h5"

        if os.path.isfile(train_path):
            train_h5 = h5py.File(train_path, 'r')

            for input_name in self._input_names_:
                if not input_name in train_h5:
                    logging.error("The HDF5 file '" + os.path.abspath(train_path) + "' has to contain the dataset "
                                  + "'" + input_name + "'")
                    sys.exit(1)

            for output_name in self._output_names_:
                if not output_name in train_h5:
                    logging.error("The HDF5 file '" + os.path.abspath(train_path) + "' has to contain the dataset "
                                  + "'" + output_name + "'")
                    sys.exit(1)

            if os.path.isfile(test_path):
                test_h5 = h5py.File(test_path, 'r')

                for input_name in self._input_names_:
                    if not input_name in test_h5:
                        logging.error("The HDF5 file '" + os.path.abspath(test_path) + "' has to contain the dataset "
                                      + "'" + input_name + "'")
                        sys.exit(1)

                for output_name in self._output_names_:
                    if not output_name in test_h5:
                        logging.error("The HDF5 file '" + os.path.abspath(test_path) + "' has to contain the dataset "
                                      + "'" + output_name + "'")
                        sys.exit(1)
            else:
                logging.warning("Couldn't load test set. File '" + os.path.abspath(test_path) + "' does not exist.")

            return train_h5, test_h5
        else:
            logging.error("Data loading failure. File '" + os.path.abspath(train_path) + "' does not exist.")
            sys.exit(1)
    """
    For stochastic transformers "lazy" flag has to be set to True in order to give
    different outputs every time.
    """
    def get_preprocess_transform(self):
        return transforms.Compose([
            # transforms.ToTensor(),
            transforms.Normalize([.485, .456, .406], [.229, .224, .225]),
            # transforms.RandomFlipLeftRight(),
        ])

    def get_train_transform(self):
        return transforms.Compose([
            # transforms.ToTensor(),
            # transforms.Normalize([.485, .456, .406], [.229, .224, .225]),
            transforms.RandomFlipLeftRight(),
        ])

class Resize2D(mx.gluon.nn.HybridBlock):
    def __init__(self, res=None, channel_axis=-1):
        super(Resize2D, self).__init__()
        self.channel_axis = channel_axis
        self.res = res

    def hybrid_forward(self, F, x, height, width):
        # height, width = kwargs['height'], kwargs['width']
        if self.channel_axis == -1:
            # (b,h,w,c) to (b,c,h,w)
            mx.ndarray.transpose(data=x, axes=(0,2,3,1))
            x = mx.nd.contrib.BilinearResize2D(data=x, height=height, width=width)
            # (b,c,h,w) to (b,h,w,c)
            mx.ndarray.transpose(data=x, axes=(0,3,2,1))
        else:
            # already (b,c,h,w)
            x = mx.nd.contrib.BilinearResize2D(data=x, height=height, width=width)
        return x

class RandomFlip2D(mx.gluon.nn.HybridBlock):
    def __init__(self, prob=0.5):
        super(RandomFlip2D, self).__init__()
        self.prob = prob

    def hybrid_forward(self, F, x):
        if np.random.randint(0,high=2):
            if len(x) == 4:
                x = x[:,:,::-1,:]
            if len(x) == 3:
                x = x[:,::-1,:]
        return x

class JitterCrop(mx.gluon.nn.HybridBlock):
    """Random Jitter to upscale and then crop back into original shape.
    Expects

    Parameters
    ----------
    dtype : str, default 'float32'
        The target data type, in string or `numpy.dtype`.
    Inputs:
        - **data**: input tensor with arbitrary shape and dtype.
    Outputs:
        - **out**: output tensor with the same shape as `data` and data type as dtype.
    """
    def __init__(self, border=20, channel_axis=-1):
        super(JitterCrop, self).__init__()
        self.resize = Resize2D(channel_axis=channel_axis)
        self.border = 20

    def hybrid_forward(self, F, x):
        height, width = x.shape[-3:-1]
        print((height, width))
        rand_h = np.random.randint(low=0, high=self.border)
        rand_w = np.random.randint(low=0, high=self.border)

        out = self.resize(x, height+self.border, width+self.border)

        print(out.shape)
        # TODO check if this is right
        print((rand_h, (height+rand_h)))
        out = F.slice(data=out, begin=(None,rand_h,rand_w,None), end=(None,(height+rand_h),(width+rand_w),None))
        return out

def test_data_loader():
    data_loader = CNNDataLoader_cNNSegment_connector_predictor()
    preproc_lib = "CNNPreprocessor_cNNSegment_connector_predictor_executor"
    train_iter, test_iter, data_mean, data_std, train_images, test_images = data_loader.load_preprocessed_data(4, preproc_lib, shuffle=False)

def test_jitter():
    jitter = JitterCrop(border=20)
    flip = transforms.Compose([# transforms.ToTensor(),
                               # transforms.Normalize([.485, .456, .406], [.229, .224, .225]),
                               RandomFlip2D(),
                               ])

    with h5py.File('/home/jt529748/git/crfrnn/python-scripts/pix2pix_datasets/datasets/facades/test.h5', 'r') as fh5:
        train_data = np.array(fh5['data'])
        # train_data = np.transpose(train_data, (0,2,3,1))
        train_label = np.array(fh5['target'])
        # train_label = np.transpose(train_label, (0,2,3,1))
    train_data = mx.nd.array(train_data)
    train_label = mx.nd.array(train_label)

    train_iter = mx.io.NDArrayIter(data=train_data,
                                   label=train_label,
                                   batch_size=32,
                                   shuffle=False)
    # print(transformer(train_data))

    # for batch in train_iter:
    #     transformer(batch)
    for batch_i, batch in enumerate(train_iter):
        labels = [batch.label[i].as_in_context(mx.cpu()) for i in range(1)]

        data_ = batch.data[0].as_in_context(mx.cpu())
        data_transformed = jitter(data_)
        print(data_transformed.shape)
        for label in labels:
            print(label.shape)

if __name__ == '__main__':
    test_jitter()