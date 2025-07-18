# (c) https://github.com/MontiCore/monticore  
import h5py
import numpy as np

path = "dataset.h5"

num_images = 100
num_chunks = 100
chunk_size = num_images // num_chunks

#targets_pred = targets_real + np.random.normal(0,0.2,(num_images,))

with h5py.File(path, 'w') as f:
    data = f.create_dataset("data",(num_images,480,640,3), maxshape=(None,480,640,3), dtype="int8")

    for i in range(num_chunks):
        print("\rChunk:",i,"/",num_chunks, end='')
        data[chunk_size*i:chunk_size*i+chunk_size] = np.random.randint(0,255,(chunk_size,480,640,3))

    targets_real = 2*np.sin(np.linspace(0,5,num_images))+6*np.cos(np.linspace(0,5,num_images))
    f.create_dataset("ergebnis_label", data=targets_real)
