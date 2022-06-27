# (c) https://github.com/MontiCore/monticore
import logging
import os
import h5py
import sys
import numpy as np
from abc import ABCMeta, abstractmethod

class AbstractDataset(metaclass=ABCMeta):
    @abstractmethod
    def __getitem__(self, index):
        pass

    @abstractmethod
    def __len__(self):
        pass

class HDF5Dataset(AbstractDataset,metaclass=ABCMeta):

    def __init__(self, data_path, input_names, output_names, output_shapes, transform=None):
        super().__init__()
        self._input_names_ = input_names
        self._output_names_ = output_names
        self._output_shapes_ = output_shapes
        self._data_path_ = data_path
        self._data_h5_ = self.load_h5_files()
        self.length = self._data_h5_[self._input_names_[0]].shape[0]
        self.transform = transform

    def __getitem__(self,index):

        data = self._data_h5_[self._input_names_[0]][index]

        if self.transform:
            data = self.transform(data)
        else:
            data = torch.from_numpy(data)

        # get label
        label = self._data_h5_[self._output_names_[0]][index]
        label = torch.from_numpy(np.asarray(label)).long()
        return (data, label)

    def __len__(self):
        return self.length

    def load_h5_files(self):
        data_h5 = None
        if os.path.isfile(self._data_path_):
            data_h5 = h5py.File(self._data_path_, 'r')
            if not (self._input_names_[0] in data_h5 and self._output_names_[0] in data_h5):
                logging.error("The HDF5 file '" + os.path.abspath(self._data_path_) + "' has to contain the datasets: "
                              + "'" + str(self._input_names_) + "', '" + str(self._output_names_) + "'")
                sys.exit(1)
            return data_h5
        else:
            logging.error("Data loading failure. File '" + os.path.abspath(self._data_path_) + "' does not exist.")
            sys.exit(1)