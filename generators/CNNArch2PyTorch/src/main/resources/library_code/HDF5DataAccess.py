import logging
import os
import h5py
import sys
import numpy as np
import torch

class HDF5DataAccess():

    def __init__(self, data_source, schemaApi):
        self._data_source = data_source
        self._schemaApi_ = schemaApi

    def load_h5_files(self, data_path):
        if os.path.isfile(data_path):
            data_h5 = h5py.File(data_path, 'r')
            return data_h5
        else:
            logging.error("Data loading failure. File '" + os.path.abspath(data_path) + "' does not exist.")
            sys.exit(1)

    def execute(self):
        train_path = self._data_source + "train.h5"
        test_path = self._data_source + "test.h5"
        dataset_train = LoadDataset(train_path)
        dataset_test = LoadDataset(test_path)
        loader_params_train = {'batch_size': self._schemaApi_.getBatch_size(), 'shuffle': True, 'num_workers': 0}
        loader_params_test = {'batch_size': self._schemaApi_.getBatch_size(), 'shuffle': False,'num_workers': 0}
        
        train_loader = torch.utils.data.DataLoader(dataset_train, **loader_params_train)
        test_loader = torch.utils.data.DataLoader(dataset_test, **loader_params_test)
        return train_loader, test_loader
    
class LoadDataset(HDF5DataAccess):
    def __init__(self, data_path):
        self.data_path = data_path
        self._data_h5_ = self.load_h5_files(self.data_path)
        self.length =  len(self._data_h5_[list(self._data_h5_.keys())[0]])
        print(self.length)

    def __getitem__(self,index):
        #data
        data = self._data_h5_[list(self._data_h5_.keys())[0]][index]
        data = torch.from_numpy(data)
        #label
        label = self._data_h5_[list(self._data_h5_.keys())[1]][index]
        label = torch.from_numpy(np.asarray(label)).long()
        
        return (data, label)

    def __len__(self):
        return self.length    