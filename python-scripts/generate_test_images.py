import h5py
import cv2
import os

save_dir = '../resources/images_fashion2x2/'

with h5py.File('../resources/fashionmnist_2x2/test.h5', 'r') as fh5:
    images = fh5['data']
    for i in range(10):
        save_path = os.path.join(save_dir, '{}.png'.format(i))
        print('Saving to... '+ os.path.abspath(save_path))
        cv2.imwrite(save_path, images[i][0])
