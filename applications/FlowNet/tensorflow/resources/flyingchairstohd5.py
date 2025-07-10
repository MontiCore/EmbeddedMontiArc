# (c) https://github.com/MontiCore/monticore  
#!/usr/share/python3

# Authors: Tim Schupp & Julian Steinsberger-Duehrssen

import IO
import os
import h5py
import numpy as np
import sys
import time as t
import cv2

# dataset size: 22872
number_to_read = 22872
save_intervall = 500

generate_downsampled_targets = True

location = ""

# Args: directory of data set
if (len(sys.argv) == 2):
    location = sys.argv[1]
else:
    location = "/FlyingChairs_release/"

number_to_read -= 1

f = open(location + "FlyingChairs_train_val.txt", "r")
split = f.read().split()

train_size = split[:number_to_read].count('1')
test_size = split[:number_to_read].count('2')
# print((train_size, test_size))


train = h5py.File(location + "/train.h5", "w")
test = h5py.File(location + "/test.h5", "w")

dset_data1_train = train.create_dataset('data_0', shape=(train_size, 3, 384, 512))
dset_data2_train = train.create_dataset('data_1', shape=(train_size, 3, 384, 512))
if not generate_downsampled_targets:
    dset_target_orig_train = train.create_dataset('target_label', shape=(train_size, 2, 384, 512))
else:
    dset_target_0_train = train.create_dataset('target_0_label', shape=(train_size, 2, 96, 128))
    dset_target_1_train = train.create_dataset('target_1_label', shape=(train_size, 2, 48, 64))
    dset_target_2_train = train.create_dataset('target_2_label', shape=(train_size, 2, 24, 32))
    dset_target_3_train = train.create_dataset('target_3_label', shape=(train_size, 2, 12, 16))
    dset_target_4_train = train.create_dataset('target_4_label', shape=(train_size, 2, 6, 8))

dset_data1_test = test.create_dataset('data_0', shape=(test_size, 3, 384, 512))
dset_data2_test = test.create_dataset('data_1', shape=(test_size, 3, 384, 512))
if not generate_downsampled_targets:
    dset_target_orig_test = test.create_dataset('target_label', shape=(test_size, 2, 384, 512)) 
else:
    dset_target_0_test = test.create_dataset('target_0_label', shape=(test_size, 2, 96, 128))
    dset_target_1_test = test.create_dataset('target_1_label', shape=(test_size, 2, 48, 64))
    dset_target_2_test = test.create_dataset('target_2_label', shape=(test_size, 2, 24, 32))
    dset_target_3_test = test.create_dataset('target_3_label', shape=(test_size, 2, 12, 16))
    dset_target_4_test = test.create_dataset('target_4_label', shape=(test_size, 2, 6, 8))
    
# print(dset_data1_train.shape)
# print(dset_data2_train.shape)
# print(dset_target_train.shape)


start = t.time()

imgs1_train = []
imgs2_train = []
target_0_train = []
target_1_train = []
target_2_train = []
target_3_train = []
target_4_train = []
imgs1_test = []
imgs2_test = []
target_0_test = []
target_1_test = []
target_2_test = []
target_3_test = []
target_4_test = []

train_ind = 0
test_ind = 0

for i in range(1, number_to_read + 1):
    name_flo = "{}_flow.flo".format(format(i, '05d'))
    name_img1 = "{}_img1.ppm".format(format(i, '05d'))
    name_img2 = "{}_img2.ppm".format(format(i, '05d'))

    # we have to transpose as the emadl framework expects channels first
    #(yes even thoug we permute it back to channels last internally for the tensorflow backend)
    temp1 = np.transpose(IO.read("{}{}".format(location + "data/", name_img1)), axes=[2, 0, 1])
    temp2 = np.transpose(IO.read("{}{}".format(location + "data/", name_img2)), axes=[2, 0, 1])

    temp_target_orig = IO.read("{}{}".format(location + "data/", name_flo)) * 0.05
    if generate_downsampled_targets:
        temp_target_0 = cv2.resize(temp_target_orig, (128, 96), interpolation=cv2.INTER_LINEAR)
        temp_target_1 = cv2.resize(temp_target_orig, (64, 48), interpolation=cv2.INTER_LINEAR)
        temp_target_2 = cv2.resize(temp_target_orig, (32, 24), interpolation=cv2.INTER_LINEAR)
        temp_target_3 = cv2.resize(temp_target_orig, (16, 12), interpolation=cv2.INTER_LINEAR)
        temp_target_4 = cv2.resize(temp_target_orig, (8, 6), interpolation=cv2.INTER_LINEAR)


    if generate_downsampled_targets:
        temp_target_0 = np.transpose(temp_target_0, axes=[2, 0, 1])
        temp_target_1 = np.transpose(temp_target_1, axes=[2, 0, 1])
        temp_target_2 = np.transpose(temp_target_2, axes=[2, 0, 1])
        temp_target_3 = np.transpose(temp_target_3, axes=[2, 0, 1])        
        temp_target_4 = np.transpose(temp_target_4, axes=[2, 0, 1])
    else:
        temp_target_orig = np.transpose(temp_target_orig, axes=[2, 0, 1])
        
        
    if split[i] == '1':
        imgs1_train.append(temp1)
        imgs2_train.append(temp2)

        if generate_downsampled_targets:
            target_0_train.append(temp_target_0)
            target_1_train.append(temp_target_1)
            target_2_train.append(temp_target_2)
            target_3_train.append(temp_target_3)
            target_4_train.append(temp_target_4)
        else:
            target_0_train.append(temp_target_orig)            
        train_ind += 1
    else:
        imgs1_test.append(temp1)
        imgs2_test.append(temp2)
        
        if generate_downsampled_targets:
            target_0_test.append(temp_target_0)
            target_1_test.append(temp_target_1)
            target_2_test.append(temp_target_2)
            target_3_test.append(temp_target_3)
            target_4_test.append(temp_target_4)
        else:
            target_0_test.append(temp_target_orig)  
        test_ind += 1

    if (i % save_intervall == 0) or (i == number_to_read):
        intv_train = len(target_0_train)
        intv_test = len(target_0_test)

        dset_data1_train[(train_ind - intv_train):train_ind] = np.array(imgs1_train)
        dset_data2_train[(train_ind - intv_train):train_ind] = np.array(imgs2_train)
        
        if generate_downsampled_targets:
            dset_target_0_train[(train_ind - intv_train):train_ind] = np.array(target_0_train)
            dset_target_1_train[(train_ind - intv_train):train_ind] = np.array(target_1_train)
            dset_target_2_train[(train_ind - intv_train):train_ind] = np.array(target_2_train)
            dset_target_3_train[(train_ind - intv_train):train_ind] = np.array(target_3_train)
            dset_target_4_train[(train_ind - intv_train):train_ind] = np.array(target_4_train)  
        else:
            dset_target_orig_train[(train_ind - intv_train):train_ind] = np.array(target_0_train) 
            

        dset_data1_test[(test_ind - intv_test):test_ind] = np.array(imgs1_test)
        dset_data2_test[(test_ind - intv_test):test_ind] = np.array(imgs2_test)
        
        if generate_downsampled_targets:
            dset_target_0_test[(test_ind - intv_test):test_ind] = np.array(target_0_test)
            dset_target_1_test[(test_ind - intv_test):test_ind] = np.array(target_1_test)
            dset_target_2_test[(test_ind - intv_test):test_ind] = np.array(target_2_test)
            dset_target_3_test[(test_ind - intv_test):test_ind] = np.array(target_3_test)
            dset_target_4_test[(test_ind - intv_test):test_ind] = np.array(target_4_test)
        else:
            dset_target_orig_test[(test_ind - intv_test):test_ind] = np.array(target_0_test) 

        print(str(i - (intv_train + intv_test)) + " - " + str(i - 1) + " written")

        imgs1_train = []
        imgs2_train = []
        target_0_train = []
        target_1_train = []
        target_2_train = []
        target_3_train = []
        target_4_train = []
        imgs1_test = []
        imgs2_test = []
        target_0_test = []
        target_1_test = []
        target_2_test = []
        target_3_test = []
        target_4_test = []        
        
train.close()
test.close()

end = t.time()

print('\nElapsed time:' + str((end - start)) + 's')
