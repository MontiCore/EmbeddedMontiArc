<#-- (c) https://github.com/MontiCore/monticore -->
<#-- So that the license is in the generated file: -->
# (c) https://github.com/MontiCore/monticore
import gc, os, json, argparse, logging
import h5py
import numpy as np
import random
from skimage.transform import rotate, rescale
import cv2


def load_data(file):
    print('\nLoading Data \t........ ', end='', flush=True)
    data = h5py.File(file, "r")

    print('Done')
    return np.array(data["data"]), np.array(data["softmax_label"])


def load_config(file):
    print('Loading Config \t........ ', end='', flush=True)
    # reading in config file

    <#list configurations as config>
    <#if (config.cleaning)??>
    cleaning='${config.cleaningName}'
    cleaning_params={
    <#if (config.cleaningParameters)??>
    <#list config.cleaningParameters?keys as param>
        '${param}': ${config.cleaningParameters[param]}<#sep>,
    </#list>
    </#if>
    }
    </#if>

    <#if (config.dataImbalance)??>
    data_imbalance='${config.dataImbalanceName}'
    data_imbalance_params={
    <#if (config.dataImbalanceParameters)??>
    <#list config.dataImbalanceParameters?keys as param>
        <#if param == "rotation_angle">
        '${param}': [<#list config.dataImbalanceParameters[param] as lr>${lr}<#if lr?has_next>, </#if></#list>],
        <#else>
        '${param}': ${config.dataImbalanceParameters[param]}<#sep>,
        </#if>
    </#list>
    </#if>
    }
    </#if>

    <#if (config.dataSplitting)??>
    data_splitting='${config.dataSplittingName}'
    data_splitting_params={
    <#if (config.dataSplittingParameters)??>
    <#list config.dataSplittingParameters?keys as param>
        <#if param == "tvt_ratio">
        '${param}': [<#list config.dataSplittingParameters[param] as r>${r}<#if r?has_next>, </#if></#list>],
        <#else>
        '${param}': ${config.dataSplittingParameters[param]}<#sep>,
        </#if>
    </#list>
    </#if>
    }
    </#if>
    </#list>

    print('Done')
    return cleaning, cleaning_params, data_imbalance, data_imbalance_params, data_splitting, data_splitting_params


# ------ cleaning algorithm ------
def clean_data(data, label, cleaning, param):
    print('Cleaning Data \t........ ', end='', flush=True)
    params = dict(param)

    # begin identifying indeces of np array to be removed 
    ## with missing and noisy
    missing, noisy = get_dirty(data, label, params)

    # remove elements by identified indices
    indices = np.append(missing, noisy)
    if np.any(indices):
        data = np.delete(data, indices, axis=0)
        label = np.delete(label, indices, axis=0)

    ## unique values
    unique = []
    if params['duplicate']:
        unique = get_unique(data, label)

    if np.any(unique):
        data = np.take(data, unique, axis=0)
        label = np.take(label, unique, axis=0)

    # remove unused arrays from memory
    del missing, noisy, indices, unique
    gc.collect()

    print('Done')
    return data, label


# ------ helper functions for data cleaning ------
def get_dirty(data, label, params):
    missing, noisy = [], []
    if params['missing']:
        """idenfify indeces of missing values"""
        # missing values (e.g., nan in data or label)
        missing = np.concatenate((
            np.unique(np.where(np.isnan(np.squeeze(data, axis=1)).all(axis=2))[0]), 
            np.where(np.isnan(label))[0]
        ),axis=0)

    if params['noisy']:
        """idenfify indeces of faulty values"""
        # faulty values (e.g. empty images or labels outside range between [0,9])
        noisy = np.concatenate((
            np.where([(data[i][0] == 0).all() for i in range(len(data))])[0],
            np.where(label < 0)[0], 
            np.where(label > 9)[0]
        ),axis=0)
    return np.sort(missing), np.sort(noisy)

def get_unique(data, label):
    """idenfify indeces of unique values"""
    arr = [tuple(row) for row in data]
    _, indx = np.unique(arr, axis=0, return_index = True)
    return np.sort(indx, axis=None)




# ------ data imbalance upsampling algorithm ------
def data_augmentation(data, label, params):
    print('Balance Data \t........ ', end='', flush=True)

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


                # rescale in 
                if params['scale_in']:
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
    
    print('Done')
    return data, label


# ------ data tvt splitting algorithm ------
def tvt_split(data, label, type, params):
    print('Splitting Data \t........ ', end='', flush=True)

    ratio = [int(i)/100 for i in params['tvt_ratio']]
    amount = [np.round((int(i)/100)*len(data)).astype(int) for i in params['tvt_ratio']]

    if type == 'srs':
        indx = np.arange(len(data))
        np.random.shuffle(indx)
        s1, s2, s3 = indx[:amount[0]], indx[amount[0]:amount[0]+amount[1]], indx[amount[0]+amount[1]:]

    elif type == 'strs':
        strata_groups = []
        for i in range(10):
            indx = np.where(label == i)[0]
            np.random.shuffle(indx)
            strata_groups.append(indx)

        # iterate over each strata and split index into tvt
        s1, s2, s3 = [],[],[]
        for i, indx in enumerate(strata_groups):
            nums = [np.round(r*len(indx)).astype(int) for r in ratio]
            if sum(nums) != len(indx): nums[0] = nums[0] + (len(indx)-sum(nums))
            s1.extend(indx[ : nums[0]])
            s2.extend(indx[nums[0] : nums[0]+nums[1]])
            s3.extend(indx[nums[0]+nums[1] : nums[0]+nums[1]+nums[2]])

    elif type == 'cs':
        # first identify ratio of label for each cluster
        (unique, counts) = np.unique(label, return_counts=True)
        label_ratio = [np.round(counts[i]/len(data)*params['sample_size']).astype(int) for i in range(len(counts))]

        # identify indeces of each label
        indeces = []
        for i in range(10): 
            arr = np.where(label==i)[0]
            np.random.shuffle(arr)
            indeces.append(arr)

        # create clusters consisting of indeces of data elements in it
        cluster, c_len = [], np.round(len(label)/params['sample_size']).astype(int)
        for i in range(c_len):
            curr = []
            for j, l in enumerate(label_ratio):
                if l != 0:
                    for item in indeces[j][0:l]: curr.append(item)  # append first l elements
                    indeces[j] = indeces[j][l:]                     # remove first l elements
            cluster.append(curr)

        # check if cluster sizes are as defined
        for num, c in enumerate(cluster):
            while len(c) != params['sample_size']:
                arr = [i for i, j in enumerate(indeces) if j.any()]
                if arr: it = random.choice(arr)
                else: break
                c = np.append(c, indeces[it][0])
                indeces[it] = indeces[it][1:]
                cluster[num] = c

        # now chose cluster for tvt split
        np.random.shuffle(cluster)
        sp1, sp2 = int(ratio[0]*len(cluster)), int(ratio[0]*len(cluster) + ratio[1]*len(cluster))

        s1 = [j for sub in cluster[:sp1] for j in sub]
        s2 = [j for sub in cluster[sp1:sp2] for j in sub]
        s3 = [j for sub in cluster[sp2:] for j in sub]

    else:
        print('ERROR: TVT sampling method unknown!')

    # take elements based on index
    train_data, train_label = np.take(data, s1, axis=0), np.take(label, s1)
    val_data, val_label = np.take(data, s2, axis=0), np.take(label, s2)
    test_data, test_label = np.take(data, s3, axis=0), np.take(label, s3)

    print('Done')

    return train_data, train_label, val_data, val_label, test_data, test_label


# ------ transform numpy array to hdf5 ------
def numpy_to_hdf5(train_data, train_label, val_data, val_label, test_data, test_label, folder):
    print('Saving Data \t........ ', end='', flush=True)

    # check if old files exist
    if os.path.exists(folder+'train.h5'): os.remove(folder+'train.h5')
    if os.path.exists(folder+'test.h5'): os.remove(folder+'test.h5')

    # create and save as h5 file
    with h5py.File(folder+"train.h5", "w") as f:
        f.create_dataset("data", train_data.shape, dtype='f4', data=train_data)
        f.create_dataset("softmax_label", train_label.shape, dtype='f4', data=train_label)

    with h5py.File(folder+"test.h5", "w") as f:
        f.create_dataset("data", test_data.shape, dtype='f4', data=test_data)
        f.create_dataset("softmax_label", test_label.shape, dtype='f4', data=test_label)
    
    if val_data is not None and val_label is not None:
        if os.path.exists(folder+'validation.h5'): os.remove(folder+'validation.h5')

        with h5py.File(folder+"validation.h5", "w") as f:
            f.create_dataset("data", val_data.shape, dtype='f4', data=val_data)
            f.create_dataset("softmax_label", val_label.shape, dtype='f4', data=val_label)

    print('Done')


if __name__ == '__main__':

    print('\n------ START run.py ------')

    # Init parser
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--input', 
        type=str, required=True, 
        help='Name of input data of type hdf5.'
    )
    
    args = parser.parse_args()

    # reading in mnist data
    _data_, _label_ = load_data(args.input)

    # loading configuration parameter
    cleaning, cleaning_params, data_imbalance, data_imbalance_params, data_splitting, data_splitting_params = load_config()

    # clean data
    data, label = clean_data(_data_, _label_, cleaning, cleaning_params)

    # check data imbalance 
    data, label = data_augmentation(data, label, data_imbalance_params)

    # split data into tvt
    train_data, train_label, val_data, val_label, test_data, test_label = tvt_split(data, label, data_splitting, data_splitting_params)

    # save it into /...
    _data_dir_ = '/root/.m2/repository/de/monticore/lang/monticar/datasets/mnist/1/mnist-1-dataset/training_data/'
    numpy_to_hdf5(train_data, train_label, val_data, val_label, test_data, test_label, _data_dir_)

    print('\nSuccessful.')