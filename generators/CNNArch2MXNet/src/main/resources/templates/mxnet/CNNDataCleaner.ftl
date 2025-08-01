<#-- (c) https://github.com/MontiCore/monticore -->
<#-- So that the license is in the generated file: -->
# (c) https://github.com/MontiCore/monticore
import gc
import logging
import warnings
import h5py
import numpy as np
import random
from skimage.transform import rotate, rescale
import cv2
import mxnet as mx
from mxnet import gluon, nd

class ${tc.fileNameWithoutEnding}:

    def __init__(self):
        self._data_dir = "/"
        self._input_names_ = [${tc.join(tc.architectureInputs, ",", "'", "'")}]
        self._output_names_ = [${tc.join(tc.architectureOutputs, ",", "'", "_label'")}]
        self._input_data_names_ = [<#list tc.architectureInputs as inputName>'${inputName?keep_before_last("_")}'<#sep>, </#list>]
        self._output_data_names_ = [${tc.join(tc.architectureOutputs, ",", "'", "label'")}]
        
    # ------ cleaning algorithm ------
    def clean_data( self, train_data, test_data, cleaning, cleaning_param ):
        cleaning_params = dict(cleaning_param)

        # transform h5 dataset to numpy array
        label = train_data[self._output_data_names_[0]]
        train_label = np.array(label)
        train_data = np.array(train_data[self._input_data_names_[0]])

        label = test_data[self._output_data_names_[0]]
        test_label = np.array(label)
        test_data = np.array(test_data[self._input_data_names_[0]])

        # begin identifying indeces of np array to be removed 
        missing_train, noisy_train = self.get_dirty(train_data, train_label, cleaning_params)
        missing_test, noisy_test = self.get_dirty(test_data, test_label, cleaning_params)
        <#--  ## with missing 
        missing_train, missing_test = [],[]
        if cleaning_params['missing']:
            missing_train = self.get_missing(train_data, train_label)
            missing_test = self.get_missing(test_data, test_label)

        ## faulty values
        noisy_train, noisy_test = [],[]
        if cleaning_params['noisy']:
            noisy_train = self.get_noisy(train_data, train_label)
            noisy_test = self.get_noisy(test_data, test_label)  -->

        # remove elements by identified indices
        indices_train = np.append(missing_train, noisy_train).astype(int)
        if np.any(indices_train):
            train_data = np.delete(train_data, indices_train, axis=0)
            train_label = np.delete(train_label, indices_train, axis=0)

        indices_test = np.append(missing_test, noisy_test).astype(int)
        if np.any(indices_test):
            test_data = np.delete(test_data, indices_test, axis=0)
            test_label = np.delete(test_label, indices_test, axis=0)

        # remove unused arrays from memory
        del missing_train, missing_test, noisy_train, noisy_test, indices_train, indices_test
        gc.collect()

        ## unique values
        unique_train, unique_test = [],[]
        if cleaning_params['duplicate']:
            unique_train = self.get_unique(train_data, train_label)
            unique_test = self.get_unique(test_data, test_label) 

        if np.any(unique_train):
            train_data = np.take(train_data, unique_train, axis=0)
            train_label = np.take(train_label, unique_train, axis=0)

        if np.any(unique_test):
            test_data = np.take(test_data, unique_test, axis=0)
            test_label = np.take(test_label, unique_test, axis=0)

        return train_data, train_label, test_data, test_label

    # ------ helper functions for data cleaning ------
    def get_dirty(self, data, label, params):
        missing, noisy = [], []
        if params['missing']:
            """idenfify indeces of missing values"""
            # missing values (e.g., nan in data or label)
            missing = np.concatenate((
                np.unique(np.where(np.isnan(np.squeeze(data, axis=1)).all(axis=2))[0]), 
                np.where(np.isnan(label))[0]
            ),axis=0)
            missing = np.sort(missing)
        
        if params['noisy']:
            """idenfify indeces of faulty values"""
            # faulty values (e.g. empty images or labels outside range between [0,9])
            noisy = np.concatenate((
                np.where([(data[i][0] == 0).all() for i in range(len(data))])[0],
                np.where(label < 0)[0], 
                np.where(label > 9)[0]
            ),axis=0)
            noisy = np.sort(noisy)
        
        return missing, noisy

    def get_unique(self, data, label):
        """idenfify indeces of unique values"""
        arr = [tuple(row) for row in data]
        _, indx = np.unique(arr, axis=0, return_index = True)
        return np.sort(indx, axis=None)

    
    # ------ data imbalance upsampling algorithm ------
    def image_augmentation(self, data, label, params):  
        _, counts = np.unique(label, return_counts=True)    # identify counts of every label
        median = int(np.median(counts))
        minority_label = np.where(counts < median-median/4)[0]
    
        if np.any(np.asarray(minority_label)):
            new_data, new_label = [], []
            # start data augmentation for every minority label
            for minority_class in minority_label:
                aug_indx = np.where(label == minority_class)[0]
                random.shuffle(aug_indx)
                iter = 0
                for i in aug_indx:
                    img = data[i][0]
                    if params['rotation_angle'] != None or params['rotation_angle'] != []:
                        for a in params['rotation_angle']: # rotation degrees
                            rotated = rotate(img,angle=a)  # shape in/out: (28,28)
                            new_data.append(np.reshape(rotated, (1,) + rotated.shape))
                            new_label.append(minority_class)
                            iter += 1

                    if params['shift']:
                        dict = {   # shift image up, down, left, right
                            (0,random.randint(1,4)): 'up',
                            (0,-random.randint(1,4)): 'down',
                            (-random.randint(1,4),0): 'left',
                            (random.randint(1,4),0): 'right' 
                        }
                        for key in dict:
                            shifted = cv2.warpAffine(img, np.float32([ [1,0,key[0]], [0,1,key[1]] ]), (28,28))  # shape in/out: (28,28)
                            new_data.append(np.reshape(shifted, (1,) + shifted.shape))
                            new_label.append(minority_class)
                            iter += 1

                    if params['scale_out']:
                        # rescale out
                        scale_fct = round(random.uniform(0.7, 0.9), 2)
                        scale_out = rescale(img, scale=scale_fct, mode='constant')
                        scale_out = cv2.warpAffine( 
                                scale_out, 
                                cv2.getRotationMatrix2D( (scale_out.shape[1],scale_out.shape[0]), 0, scale_fct), # shape in/out: (28,28)
                                (28,28)
                        )    
                        new_data.append(np.reshape(scale_out, (1,) + scale_out.shape))
                        new_label.append(minority_class)
                        iter += 1

                    if params['scale_in']:
                        # rescale in 
                        scale_fct = round(random.uniform(1.05, 1.1), 2)
                        scale_in = rescale(img, scale=scale_fct, mode='constant')
                        scale_in = cv2.warpAffine( 
                                scale_in, 
                                cv2.getRotationMatrix2D( (scale_in.shape[1],scale_in.shape[0]), 0, scale_fct), # shape in/out: (28,28)
                                (28,28)
                        ) 
                        new_data.append(np.reshape(scale_in, (1,) + scale_in.shape))
                        new_label.append(minority_class)
                        iter += 1
                    
                    # check if already enough new generated img
                    if iter >= int(median-(counts[minority_class])):
                        break

            if np.any(np.asarray(new_data)):
                data = np.concatenate((data, np.asarray(new_data)), axis=0)
                label = np.concatenate((label, np.asarray(new_label)), axis=0)
        
        return data, label

    def check_bias(self, data, label, model_dir):
        # load model 
        sym, arg_params, aux_params = mx.model.load_checkpoint(model_dir + '/model_newest',0)
        net = mx.mod.Module(symbol=sym, data_names=['data_'], label_names=['softmax__label'])
        net.bind(for_training=False, data_shapes=[('data_', (1,1,28,28))], label_shapes=[ ("softmax__label", (1,)) ])
        net.set_params(arg_params, aux_params)
        test_iter = mx.io.NDArrayIter([data],[label],
                                batch_size=1,
                                data_name='data_',
                                label_name='softmax__label')

        # calculate confusion matrix
        CM = np.zeros((10,10), dtype=int)
        #for t in range(len(data)):
        for pred, i_batch, batch in net.iter_predict(test_iter):
            i = batch.label[0].asnumpy().astype(int)[0]
            j = pred[0].asnumpy().argmax(axis=0) # get predicted result
            CM[i][j] += 1

        # check number of matrix dimensions
        sum_col = np.sum(CM, axis=0, dtype=np.int32)
        sum_row = np.sum(CM, axis=1, dtype=np.int32)
        if(np.sum(sum_col) != np.sum(sum_col)):
            logging.error("Sum of actual label and sum of predicted label is different in Confusion Matrix!")

        # start Bias Checking
        TP = CM.diagonal()
        
        TPR = np.zeros((10), dtype=float)
        for c in range(10):
            TPR[c] = np.round(CM[c][c] / (CM[c][c] + sum([CM[c][i] for i in range(0,10) if i != c])),4)

        FPR = np.zeros((10), dtype=float)
        for c in range(10):
            FP = sum([CM[i][c] for i in range(0,10) if i != c])
            FPR[c] = np.round(FP / (FP + sum([CM[i][j] for i in range(0,10) for j in range(0,10) if i != c])),4)

        # show results
        logging.info('\n--------- Check Model Bias: --------------------------------------------------\n')
        
        logging.info('TP:' + np.array2string(TP))
        logging.info('TPR:' + np.array2string(TPR))
        logging.info('FPR:' + np.array2string(FPR))

        logging.info('\n\tSatisfy Multi-class Demographic Parity \t...... ' + str(np.all(np.isclose(TP, TP[0]))))
        logging.info('\tSatisfy Multi-class Equality of Odds \t...... ' + str(np.all(np.isclose(TPR, TPR[0])) and np.all(np.isclose(FPR, FPR[0]))))
        logging.info('\tSatisfy Multi-class Equal Opportunity \t...... ' + str(np.all(np.isclose(TPR, TPR[0]))))

        logging.info('\n------------------------------------------------------------------------------')


    # ------ transform numpy array to hdf5 ------
    def numpy_to_hdf5(self, train_data, train_label, test_data, test_label):
        # create and save as h5 file
        with h5py.File("_train.h5", "w") as f:
            f.create_dataset(self._input_data_names_[0], train_data.shape, dtype='f4', data=train_data)
            f.create_dataset(self._output_data_names_[0], train_label.shape, dtype='f4', data=train_label)

        with h5py.File("_test.h5", "w") as f:
            f.create_dataset(self._input_data_names_[0], test_data.shape, dtype='f4', data=test_data)
            f.create_dataset(self._output_data_names_[0], test_label.shape, dtype='f4', data=test_label)

        # have to return data of type <class 'h5py._hl.files.File'>
        return h5py.File("_train.h5", "r"), h5py.File("_test.h5", "r")
