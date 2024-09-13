import numpy as np
import struct
import matplotlib.pyplot as plt
import h5py
import os

def read_idx(filename):
    """ Read IDX format file. """
    with open(filename, 'rb') as f:
        # Read magic number
        magic_number = struct.unpack('>I', f.read(4))[0]

        if magic_number == 2051:
            # Image file
            num_images = struct.unpack('>I', f.read(4))[0]
            num_rows = struct.unpack('>I', f.read(4))[0]
            num_cols = struct.unpack('>I', f.read(4))[0]
            images = np.fromfile(f, dtype=np.uint8).reshape(num_images, num_rows, num_cols)
            return images
        elif magic_number == 2049:
            # Label file
            num_labels = struct.unpack('>I', f.read(4))[0]
            labels = np.fromfile(f, dtype=np.uint8)
            return labels
        else:
            raise ValueError('Unknown magic number')

# Example usage
train_images = read_idx('emnist-digits-train-images-idx3-ubyte')
train_labels = read_idx('emnist-digits-train-labels-idx1-ubyte')


nr = 9
print(len(train_images[0]))

true_images = train_images.copy()
j = 0
for image in true_images:
    Newimage = np.rot90(image, k=-1)
    imageFinal = np.fliplr(Newimage)
    true_images[j] = imageFinal
    j = j+1

plt.imshow(np.squeeze(true_images[nr]),cmap="gray")
plt.title(str(train_labels[nr]))
plt.show()

labels = [0]*10
imageDB = [[0]*100000]*10
index = 0
for label in train_labels:
    labels[label] += 1
    imageDB[label][index] = true_images[index]

print(labels)


sorted_images = [x for _, x in sorted(zip(train_labels, true_images), key=lambda pair: pair[0])]
sorted_labels = sorted(train_labels)


sorted_images = np.array(sorted_images)
sorted_labels = np.array(sorted_labels)

if not os.path.isfile("target.dataset_sorted.h5"):
        with h5py.File("target.dataset_sorted.h5", 'w') as h5_file:
            dataset1 = h5_file.create_dataset("data",(len(true_images),1,28,28),"float32")
            labelset1 = h5_file.create_dataset("softmax_label",(len(sorted_labels),),"float32")
            reshaped_images = np.expand_dims(sorted_images, axis=1)
            dataset1[:] = reshaped_images
            labelset1[:] = sorted_labels



