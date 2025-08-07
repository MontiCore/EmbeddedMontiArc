# (c) https://github.com/MontiCore/monticore  
import argparse
import h5py
from functools import reduce
from VTool import start_plot
import numpy as np
import csv
from subprocess import call
import os
import matplotlib.pyplot as plt


parser = argparse.ArgumentParser(description='Visualization of predictions for the end-to-end autonomous driving model.')
parser.add_argument('-i', type=str, help='Path to H5 container which includes data and targets.')
parser.add_argument('-p', action='store_true' , help='Set this flag in order to predict. Default is no prediction.')
parser.add_argument('-v', action='store_true' , help='Set this flag in order to visualise. Default is no visualisaton.')
parser.add_argument('-d', type=int, help='Set delay (in ms) between each image to be plotted.')
parser.add_argument('-t', action='store_true', help='Set this flag to create a plot showing both real target and prediction in a time series graph.')
args = parser.parse_args()

print("Data:",args.i)
print("Predict:",args.p)
print("Visualise:",args.v)

if not os.path.exists(args.i):
    print("Data not existent")
    quit()
    
#make predictions
if args.p:
    call(['build/src/cpp/steeringAnglePredictor',args.i,'v-tool/predictions.csv'])

if args.v:
    with h5py.File(args.i, 'r') as f:
        keys = set(f.keys())
        target_keys = {k for k in keys if k[-5:] == "label"}
        data_keys = list(keys - target_keys)
        target_keys = list(target_keys)

        print("-"*40)
        print("Data:")
        for i,k in enumerate(data_keys):
            print("  ("+str(i)+") "+k, "->", f[k].shape)
            HEIGHT=f[k].shape[2]
            WIDTH=f[k].shape[3]
        #data_key = data_keys[int(input("Pick data number: "))]
        data_key = data_keys[0]
        target_key = target_keys[0] #hard-coded: pick first target in list

        # Check data size - if it is larger than our threshold MAX_SIZE, only display the chosen chunk
        MAX_SIZE = int(2*10e7)
        data_total_size = reduce((lambda x,y: x*y), f[data_key].shape)

        if data_total_size < MAX_SIZE:
            images = np.array(f[data_key])
            images = images.reshape((-1,HEIGHT,WIDTH,3)).astype("uint8")
            targets_real = np.array(f[target_key])
        else:
            num_chunks = data_total_size // MAX_SIZE + 1
            chunk_size = f[data_key].shape[0] // num_chunks
            chunk_id = 0
            while (chunk_id < 1 or chunk_id > num_chunks):
                chunk_id = int(input("Dataset is too large. It was divided into " + str(num_chunks) + " chunks. Which chunk [1-" + str(num_chunks) + "] should be selected?: "))
            chunk_id=chunk_id-1
            images = np.array(f[data_key][chunk_size*chunk_id:chunk_size*chunk_id+chunk_size][:])
            images = images.reshape((-1,HEIGHT,WIDTH,3)).astype("uint8")
            targets_real = np.array(f[target_key][chunk_size*chunk_id:chunk_size*chunk_id+chunk_size])

        with open('v-tool/predictions.csv') as f:
             for row in csv.reader(f, delimiter=','):
                targets_pred = np.array([float(r) for r in row]).astype('float32')

        #print('predictions:',targets_pred)

        if args.d:
            delay = args.d
        else:
            delay = 0

         ### plot real target and prediction tme series 

        if args.t:
            plt.figure('Plot of real and predicted targets.')
            plt.plot(targets_real*3.141*2,c='green')
            plt.plot(targets_pred*3.141*2,c='red')
            plt.legend(['Real targets (in degrees)','Predicted targets (in degrees)'])
            plt.show()

         ###

        print("-"*40)
        print('Starting plot...')
        start_plot(images, targets_real, targets_pred, delay)

        
        print("-"*40)
