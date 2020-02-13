from time import sleep
import h5py
import cv2
import os
import numpy as np
import resource
import sys

def main():
    dir_cat = '/home/treiber/git/BA/catsanddogs/PetImages/Cat'
    dir_dog = '/home/treiber/git/BA/catsanddogs/PetImages/Dog'

    cats = [path for path in os.listdir(dir_cat) if '.jpg' in path]

    dogs = [path for path in os.listdir(dir_dog) if '.jpg' in path]

    with open('cats.txt', 'w') as fh:

        for cat in cats:
            img = cv2.imread(os.path.join(dir_cat, cat))
            if img is not None:
                img = cv2.resize(img, (500, 500))
                fh.write(cat+'\n')
            else:
                print(cat, img)
                continue



    with open('dogs.txt', 'w') as fh:

        for dog in dogs:
            img = cv2.imread(os.path.join(dir_dog, dog))
            if img is not None:
                img = cv2.resize(img, (500, 500))
                fh.write(dog+'\n')
            else:
                print(dog, img)
                continue

def memory_limit():
    soft, hard = resource.getrlimit(resource.RLIMIT_AS)
    resource.setrlimit(resource.RLIMIT_AS, (get_memory() * 1024 / 2, hard))

def get_memory():
    with open('/proc/meminfo', 'r') as mem:
        free_memory = 0
        for i in mem:
            sline = i.split()
            if str(sline[0]) in ('MemFree:', 'Buffers:', 'Cached:'):
                free_memory += int(sline[1])
    return free_memory

if __name__ == '__main__':
    memory_limit() # Limitates maximun memory usage to half
    try:
        main()
    except MemoryError:
        sys.stderr.write('\n\nERROR: Memory Exception\n')
        sys.exit(1)