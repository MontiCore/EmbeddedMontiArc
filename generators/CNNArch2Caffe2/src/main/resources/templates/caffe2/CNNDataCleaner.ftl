<#-- (c) https://github.com/MontiCore/monticore -->
<#-- So that the license is in the generated file: -->
# (c) https://github.com/MontiCore/monticore
import gc, os
import h5py
import numpy as np
import random
from skimage.transform import rotate, rescale
import cv2

# import for caffe2
import cv2
from caffe2.proto import caffe2_pb2
from caffe2.python import (
    workspace,
    brew,
    core
)

class ${tc.fileNameWithoutEnding}:

    def __init__(self):
        self._current_dir_ = os.path.join('./')

    
    # ------ cleaning algorithm ------
    def clean_data(self, data, label, cleaning, cleaning_param):
        cleaning_params = dict(cleaning_param)

        # begin identifying indeces of np array to be removed 
        ## with missing 
        missing, noisy = self.get_dirty(data, label, cleaning_params)
       
        # remove elements by identified indices
        indices = np.append(missing, noisy).astype(int)
        if np.any(indices):
            data = np.delete(data, indices, axis=0)
            label = np.delete(label, indices, axis=0)

        # remove unused arrays from memory
        del missing, noisy, indices
        gc.collect()

        ## unique values
        unique = []
        if cleaning_params['duplicate']:
            unique = self.get_unique(data, label)
        
        if np.any(unique):
            data = np.take(data, unique, axis=0)
            label = np.take(label, unique, axis=0)

        return data, label

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
    def image_augmentation(self, data, label, data_imbalance, params): 
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


    def check_bias(self, data, label, model_dir, opts):
        # Reset workspace and load model
        workspace.ResetWorkspace(os.path.join('/opt'))
        init_def = caffe2_pb2.NetDef()
        with open(model_dir+'/init_net.pb', 'rb') as f:
            init_def.ParseFromString(f.read())
            init_def.device_option.CopyFrom(opts)
            workspace.RunNetOnce(init_def.SerializeToString())
    
        net_def = caffe2_pb2.NetDef()
        with open(model_dir+'/predict_net.pb', 'rb') as f:
            net_def.ParseFromString(f.read())
            net_def.device_option.CopyFrom(opts)
            workspace.CreateNet(net_def.SerializeToString(), overwrite=True)

        # calculate confusion matrix
        CM = np.zeros((10,10), dtype=int)
        for t in range(len(data)):
            i = label[t]
            input = data[t].reshape((1,1,28,28)).astype('float32')   # reshape to (1,1,28,28)
            workspace.FeedBlob("data", input, device_option=opts)       # FeedBlob
            workspace.RunNet('deploy_net', num_iter=1)                  # Forward
            j = np.argmax(workspace.FetchBlob("softmax_"))
        
            CM[i][j] += 1

        # check number of matrix dimensions
        sum_col = np.sum(CM, axis=0, dtype=np.int32)
        sum_row = np.sum(CM, axis=1, dtype=np.int32)
        if(np.sum(sum_col) != np.sum(sum_col)):
            print("Error: Sum of actual label and sum of predicted label is different in Confusion Matrix!")

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
        print('\n--------- Check Model Bias: --------------------------------------------------\n')
        
        print('TP:' + np.array2string(TP))
        print('TPR:' + np.array2string(TPR))
        print('FPR:' + np.array2string(FPR))

        print('\n\tSatisfy Multi-class Demographic Parity \t...... ' + str(np.all(np.isclose(TP, TP[0]))))
        print('\tSatisfy Multi-class Equality of Odds \t...... ' + str(np.all(np.isclose(TPR, TPR[0])) and np.all(np.isclose(FPR, FPR[0]))))
        print('\tSatisfy Multi-class Equal Opportunity \t...... ' + str(np.all(np.isclose(TPR, TPR[0]))))

        print('\n------------------------------------------------------------------------------')

        
    