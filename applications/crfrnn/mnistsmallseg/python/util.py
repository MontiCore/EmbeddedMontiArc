"""
MIT License

Copyright (c) 2017 Sadeep Jayasumana

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import numpy as np
from PIL import Image
import cv2
import mxnet as mx

# Pascal VOC color palette for labels
_PALETTE = [0, 0, 0,
            128, 0, 0,
            0, 128, 0,
            128, 128, 0,
            0, 0, 128,
            128, 0, 128,
            0, 128, 128,
            128, 128, 128,
            64, 0, 0,
            192, 0, 0,
            64, 128, 0,
            192, 128, 0,
            64, 0, 128,
            192, 0, 128,
            64, 128, 128,
            192, 128, 128,
            0, 64, 0,
            128, 64, 0,
            0, 192, 0,
            128, 192, 0,
            0, 64, 128,
            128, 64, 128,
            0, 192, 128,
            128, 192, 128,
            64, 64, 0,
            192, 64, 0,
            64, 192, 0,
            192, 192, 0]

_IMAGENET_MEANS = np.array([123.68, 116.779, 103.939], dtype=np.float32)  # RGB mean values

def get_preprocessed_image(file_name, res=(480,480)):
    """ Reads an image from the disk, pre-processes it by subtracting mean etc. and
    returns resized RGB image.
    """

    ### load image bgr
    img = np.array(cv2.imread(file_name, cv2.IMREAD_COLOR))

    ### normalize
    img = img - _IMAGENET_MEANS[::-1]
    img = img[:, :, ::-1]  # Convert to RGB

    img_h, img_w, img_c = img.shape
    assert img_c == 3, 'Only RGB images are supported.'

    img = cv2.resize(img, res)

    return np.expand_dims(img.astype(np.float32), 0), img_h, img_w

def get_preprocessed_image_mnist(file_name, res=(28,28)):

    ### load image gray
    img = np.array(cv2.imread(file_name, cv2.IMREAD_GRAYSCALE))

    ### normalize
    img_h, img_w = img.shape

    if (img_h, img_w) != res:
        img = cv2.resize(img, res)

    img = np.expand_dims(np.expand_dims(img.astype(np.float32), 0), 0)

    return img, img_h, img_w

def get_label_image(probs, img_h, img_w):
    """ Returns the label image (PNG with Pascal VOC colormap) given the probabilities.

    Note: This method assumes 'channels_last' data format.
    """

    labels = probs.argmax(axis=1).astype('uint8').asnumpy()[0]
    h, w = labels.shape[:2]
    palette = [_PALETTE[n:n+3] for n in range(0, len(_PALETTE), 3)]
    out = np.zeros((h, w, 3))

    for i, color in enumerate(palette):
        out[labels==i] = color

    out = cv2.resize(out, (img_w, img_h))
    return out

# translates image to 1H encoding
def rgb_to_1Hlabel(_label, palette):
    n_classes = len(palette)
    teye = np.eye(n_classes, dtype=np.uint8)
    label_seg = np.zeros([_label.shape[1:], n_classes], dtype=np.uint8)

    for i, color in enumerate(palette):
        # label_seg [np.all(_label.transpose([1,2,0])==_label_dict[label],axis=-1)] = teye[i]
        label_seg [np.all(_label.transpose([1,2,0]) == color, axis=-1)] = teye[i]

    return label_seg.transpose([2,0,1])

def get_label_image_1hot(probs, img_h, img_w):
    # probs.shape == NClasses, Height, Width
    mask = np.argmax(probs,axis=0)
    return mask

def get_preprocessed_image2(file_name):
    """ Reads an image from the disk, pre-processes it by subtracting mean etc. and
    returns a numpy array that's ready to be fed into a Keras model.

    Note: This method assumes 'channels_last' data format in Keras.
    """

    img = np.array(Image.open(file_name)).astype(np.float32)
    assert img.ndim == 3, 'Only RGB images are supported.'
    img = img - _IMAGENET_MEANS
    img = img[:, :, ::-1]  # Convert to BGR
    img_h, img_w, img_c = img.shape
    assert img_c == 3, 'Only RGB images are supported.'

    pad_h = 500 - img_h
    pad_w = 500 - img_w
    img = np.pad(img, pad_width=((0, pad_h), (0, pad_w), (0, 0)), mode='constant', constant_values=0)
    return np.expand_dims(img.astype(np.float32), 0), img_h, img_w

def test_colors_labels():
    test_input = np.vstack([np.full((240,480), 10, dtype=np.uint8), np.full((240,480), 5, dtype=np.uint8)])
    img_h, img_w = test_input.shape[:2]
    labels = get_label_image(test_input, img_h, img_w)
    return labels



if __name__ == '__main__':
    labels = test_colors_labels()

    import matplotlib.pyplot as plt
    plt.imshow(labels), plt.show()
