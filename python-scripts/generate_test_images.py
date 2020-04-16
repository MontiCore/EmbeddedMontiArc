import h5py
import cv2
import os
import numpy as np


voc_mean = [.485, .456, .406]
voc_std = [.229, .224, .225]


def generate_images_fashion2x2(save_dir):
    with h5py.File('../resources/fashionmnist_2x2/test.h5', 'r') as fh5:
        images = fh5['data']
        for i in range(10):
            save_path = os.path.join(save_dir, '{}.png'.format(i))
            print('Saving to... '+ os.path.abspath(save_path))
            cv2.imwrite(save_path, images[i][0])

def generate_images_voc(save_dir):
    with h5py.File('/home/jt529748/.mxnet/datasets/voc/test.h5', 'r') as fh5:
        images = fh5['data']
        for i in range(10):
            img = images[i]
            img = np.transpose(img, (1,2,0))
            img = img[:,:,::-1]
            img = ((img * voc_std) + voc_mean) * 255
            save_path = os.path.join(save_dir, '{}.png'.format(i))
            print('Saving to... '+ os.path.abspath(save_path))
            cv2.imwrite(save_path, img)

if __name__=='__main__':
    save_dir_voc = '../resources/images_voc/'
    generate_images_voc(save_dir_voc)

    save_dir_fashion2x2 = '../resources/images_fashion2x2/'
    generate_images_fashion2x2(save_dir_fashion2x2)