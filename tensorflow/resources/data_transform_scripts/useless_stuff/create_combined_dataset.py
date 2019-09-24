from shutil import copyfile
import csv
from itertools import islice

def csv_to_dic(csv_f , idx):
    with open(csv_f, 'rb') as csv_file:
        lable_reader = islice(csv.reader(csv_file,delimiter=','), idx , None)
        dic = {}
        for row in lable_reader:
            dic[row[0]] = row[1]

    return dic

def create_sub_dataset():

    src = '/media/felixh/hdd/extracted_dataset-2-2.bag/center/'
    dst = '/media/felixh/hdd/dataset_2/selected_subset/'

    dic = csv_to_dic("selected_subset.csv",0)
    keys = dic.keys()
    keys.sort()

    for idx, key in enumerate(keys):
        print "copying file " ,idx ,"/",len(keys)
        src_file = src + key + '.jpg'
        dst_file = dst + key + '.jpg'
        copyfile(src_file, dst_file)



if __name__== "__main__":
    create_sub_dataset()
