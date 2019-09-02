import os, os.path
import glob
import csv
from itertools import islice
import h5py
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
from scipy import stats


import os

#for1475186966202305668 resizing
import cv2

#DIR Dictionary setup
#Data_Dir = '/media/felixh/hdd/extracted_dataset-2-2.bag/'
Data_Dir = '../test_set/'

center_cam = Data_Dir + 'center/'
left_cam =   Data_Dir + 'left/'
right_cam =  Data_Dir + 'right/'
lable_csv = Data_Dir + 'steering.csv'

PIXEL_WIDTH  = 640
PIXEL_HEIGHT = 480

#output number of pictures
print "Data Info  ...."

len_pictures = len([name for name in os.listdir(center_cam) if os.path.isfile(os.path.join(center_cam, name))])
print "Number pictures to format: center cam",len_pictures

start = 0
ende = len_pictures
print "---------------------------"


# helper function to acomplish parse_all_pictures
#increase
def parse_angle_picture(pic_timestamp,idx, epsilon, csv_dic, csv_list):

    pic_timestamp = int(pic_timestamp)

    distance = epsilon
    selected_ts = 0
    breakpoint = -1

    for idx, ts in enumerate(csv_list):

        ts_int = int(ts)

        diff = pic_timestamp-ts_int
        if(diff > 0  - epsilon):
            if(abs(diff) < distance):
                distance = abs(diff)
                selected_ts = ts
                breakpoint = idx
        else:
            break

    if(breakpoint == -1):
        raise Exception("Something went wrong in function parse_angle_picture")

    angle = csv_dic[selected_ts]

    return pic_timestamp,selected_ts,angle,breakpoint


    #with open(lable_csv, 'rb') as csv_file:
     #   lable_reader = islice(csv.reader(csv_file,delimiter=','), 1, None)
      #  distance = epsilon
       #  selected_row = None
         #selected_ts = 0

         #for idx,row in enumerate(lable_reader):
           #  ts_int = int(row[0])
             #diff = pic_timestamp-ts_int
             #if(diff > 0  - epsilon):
               #  if(abs(diff) < distance):
                 #    distance = abs(diff)
                   #  del row[0]
                     #selected_row = row
                     #selected_ts = ts_int
             #else:
               #  break

     #if(selected_row == None):
       #  raise Exception("No suitable match found: try increase epsilon")

     #print pic_timestamp
     #print selected_row
     #print "-------------"
     #return pic_timestamp, selected_row, distance


def csv_to_dic(csv_f , idx):

    with open(csv_f, 'rb') as csv_file:
         lable_reader = islice(csv.reader(csv_file,delimiter=','), idx , None)
         dic = {}
         for row in lable_reader:
             dic[row[0]] = row[1]

    return dic


#returns dictionary with picture idetifier (time of shot) with nearest measurment (timestamp) of steering angle
def parse_center_to_angle():

    print "Starting Step  parse_center_to_angle"
    pictures = [name for name in os.listdir(center_cam) if os.path.isfile(os.path.join(center_cam, name))]
    pictures.sort()
    pictures = pictures[start:ende]
    dic = {}
    i = 0

    csv_dic = csv_to_dic(lable_csv,1)
    csv_list = csv_dic.keys()
    csv_list.sort()

    for pic in pictures:
        pic_int = int(pic.split(".")[0])
        #epsilon default set here!!!
        pic,ts,angle,index = parse_angle_picture(pic_int, 1, 100000000, csv_dic ,csv_list)

        dic[pic] = angle
        csv_list = csv_list[index:]

        i = i + 1
        if i % 100 == 0:
            print "proccesed pictures :" , i , "/", len(pictures)

    keys = dic.keys()
    keys.sort()
    with open('center_to_angle.csv', 'w') as f:
        for key in keys:
            f.write("%s,%s\n"%(key,dic[key]))

    return dic


def center_to_hdf5(csv_data,begin,end):

    dic = csv_to_dic(csv_data,0)
    pic_ids = dic.keys()
    pic_ids.sort()

    if(begin == -1 and end == -1):
        begin = 0
        end = len(pic_ids) - 1

    pic_ids = pic_ids[begin:end]


    print "Starting Step create h5py"
    train_ids = pic_ids

    with h5py.File('train.h5', 'w') as hf:

            center_img = np.zeros([1,3,PIXEL_HEIGHT,PIXEL_WIDTH]).astype(np.float32)
            center_img[0,:,:,:] = np.reshape(np.array(Image.open(center_cam+str(pic_ids[0])+".jpg")),[3,PIXEL_HEIGHT,PIXEL_WIDTH])

            target = np.array([dic[pic_ids[0]]]).astype(np.float32)

            print target
            print "----------------"

            hf.create_dataset('data', data=center_img, maxshape=(None,3,PIXEL_HEIGHT,PIXEL_WIDTH))
            hf.create_dataset('target_label',data=target , maxshape=(None,))

            for i,img in enumerate(train_ids):
                if i != 0:
                    print "loading img :", i , "/" , len(pic_ids)-1

                    center_img = np.zeros([1,3,PIXEL_HEIGHT,PIXEL_WIDTH]).astype(np.float32)
                    center_img[0,:,:,:] = np.reshape(np.array(Image.open(center_cam+str(pic_ids[i])+".jpg")),[3,PIXEL_HEIGHT,PIXEL_WIDTH])

                    next_target = np.array([dic[pic_ids[i]]]).astype(np.float32)

                    print pic_ids[i], ":", next_target


                    hf['data'].resize((hf['data'].shape[0] + center_img.shape[0]), axis = 0)
                    hf['data'][-center_img.shape[0]:] = center_img

                    hf['target_label'].resize((hf['target_label'].shape[0] + next_target.shape[0]), axis = 0)
                    hf['target_label'][-next_target.shape[0]:] = next_target




# return dictionary with center_cam pic as key which is associated with list  [left_img,right_img,steeringangle,N.A,speed]
# NOT IN USE ANYMORE
def parse_center_to_left_right():

    print "STARTING Step  parse center to left"
    dic = parse_center_to_angle()
    pic_ids = dic.keys()
    lable_csv = Data_Dir + 'steering.csv'
    pic_ids.sort()

    right_cam_list = [int(name.split(".")[0]) for name in os.listdir(right_cam) if os.path.isfile(os.path.join(right_cam, name))]
    left_cam_list = [int(name.split(".")[0]) for name in os.listdir(left_cam) if os.path.isfile(os.path.join(left_cam, name))]

    #sort for matching
    right_cam_list.sort()
    left_cam_list.sort()
    right_cam_list = right_cam_list[start:ende]
    left_cam_list = left_cam_list[start:ende]
    #consitency check leave out in actuparse_center_to_angleall computation to save time
    i = 0
    epsilon = 100000000
    for pic in pic_ids:

        if i % 100 == 0:
            print "maparse_center_to_angletched pictures :" , i , "/", len(pic_ids)

        diff = pic - right_cam_list[i]
        diff2 = pic -left_cam_list[i]

        dic[pic].insert(0,right_cam_list[i])
        dic[pic].insert(0,left_cam_list[i])

        if diff > epsilon or diff2 > epsilon:
             raise Exception("do consitency check in parsing center to left right cam: images matched to distant from eachother")
        i = i+1

    return dic

#returns dictionary
def load_parsed_images_from_csv(v_file):
    with open(v_file, 'rb') as csv_file:
        lable_reader = islice(csv.reader(csv_file,delimiter=','), 0, None)
        dic = {}
        for row in lable_reader:
            dic[row[0]] = row[1]

    return dic


def build_slope():

    dic = load_parsed_images_from_csv("center_to_angle.csv")
    ordered_list = dic.keys()
    ordered_list.sort()

    save_dic = {}

    for idx, key  in enumerate(ordered_list):
        if(idx >= 5 and idx <= len(ordered_list)-5):
            print "sliding windowd from ", idx -5 , "to : " , idx+5
            x = range(0,10)
            keys_for_regression = ordered_list[idx -5:idx + 5]
            y = np.array([dic[n] for n in keys_for_regression]).astype(np.float)
            slope, intercept, r_value, p_value, std_err = stats.linregress(x,y)
            print "key: " , key
            print "slope :" , slope
            save_dic[key] = slope
            print "================================="

    keys = save_dic.keys()
    keys.sort()
    with open('center_to_slope.csv', 'w') as f:
        for key in keys:
            f.write("%s,%s\n"%(key,save_dic[key]))


def select_subset_of_turns():

    dic_angle = load_parsed_images_from_csv("center_to_angle.csv")
    dic_slope = load_parsed_images_from_csv("center_to_slope.csv")

    keys = dic_slope.keys()
    keys.sort()

    return_dic = {}
    for key in keys:
        angle = float(dic_angle[key])
        slope = float(dic_slope[key])

        if(abs(slope) >= 0.01 or abs(angle) >= 0.5):
            return_dic[key] = angle

    keys = return_dic.keys()
    keys.sort()

    with open('selected_subset.csv', 'w') as f:
        for key in keys:
            f.write("%s,%s\n"%(key,return_dic[key]))


if __name__== "__main__":

    parse_center_to_angle()
    center_to_hdf5("center_to_angle.csv",-1,-1)
    #build_slope()
    #select_subset_of_turns()



