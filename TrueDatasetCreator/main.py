import h5py
import numpy as np
import random


def getDatasets(filePath):
    with h5py.File(filePath,"r") as h5file:
        set1 = h5file["data"]
        labelSet = h5file["softmax_label"]
        #Return as nparrays
        data_array = np.array(set1)
        labels_array = np.array(labelSet)
        return data_array, labels_array

resSize = 10000

filePath = "target.dataset_sorted.h5"

data1, labels1 = getDatasets(filePath)

filePath = "target.dataset_alpha_sorted.h5"

data2, labels2 = getDatasets(filePath)

#Digit Dataset
print(len(data1))

#Lexical Dataset
print(len(data2))


result = np.zeros((resSize, 1, 28, 28))
resLabels = np.zeros(resSize)


#Use only A-F from the Lexical Dataset (6/26)
#Dataset should be evenly distributed
data2 = data2[0:int(len(data2)*6/26)-1]
labels2 = labels2[0:int(len(labels2)*6/26)-1]

assert len(data2) == len(labels2)
assert len(data1) == len(labels1)

for i in range(0, resSize):
    #6/16 chance to choose a letter
    n = random.randint(0, 15)
    if n > 9:
        choose = random.randint(0, len(data2)-1)
        result[i] = data2[choose]
        resLabels[i] = labels2[choose]+9 #A has a label of 1 but it shall be 10 etc.
    else:
        choose = random.randint(0, len(data1)-1)
        result[i] = data1[choose]
        resLabels[i] = labels1[choose]

#Output
with h5py.File("target.output.h5","w") as oFile:
    set1 = oFile.create_dataset("data", (resSize,1,28,28), np.float32)
    labelSet = oFile.create_dataset("softmax_label", (resSize,), np.float32)
    set1[:] = result
    labelSet[:] = resLabels










