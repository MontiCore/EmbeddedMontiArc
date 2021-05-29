# (c) https://github.com/MontiCore/monticore  
import os, os.path
import glob
import csv
from itertools import islice
import h5py
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
from scipy import stats
import random
import sys
import math
import os
import cv2
import argparse


parser = argparse.ArgumentParser(description='Visualization of predictions for the end-to-end autonomous driving model.')
parser.add_argument('-i', type=str, help='Path to training_image, the folder in which all images are stroed needs to be named center (udacity extraction format)')
parser.add_argument('-n', type=int, help='Number of images used to generate image')
parser.add_argument('-a',action='store_true' , help='Set this flag in order to generate h5 file based on all images ')
parser.add_argument('-c', action='store_true' , help='Set this flag in order to generate h5 file only curves from images')
parser.add_argument('-l', action='store_true' , help='Set this flag in order to generate only straight images')
parser.add_argument('-s', type=float, help='Set this flag in order to generate h5 where specified float indicates percentage of straigth data all other data is curves, percentage cannot exceed 0.5 ')
parser.add_argument('-r', type=str , help='high_res image 640/480: string specifies csv file')
args = parser.parse_args()

if not os.path.exists(args.i):
     print("Data not existent")
     quit()

Data_Dir = args.i

center_cam = Data_Dir + 'center/'
left_cam =   Data_Dir + 'left/'
right_cam =  Data_Dir + 'right/'
lable_csv = Data_Dir + 'steering.csv'

print center_cam

PIXEL_WIDTH  = int(640/4)
PIXEL_HEIGHT = int(480/4)

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

    angle = csv_dic[selected_ts][0]
    speed = csv_dic[selected_ts][1]

    return pic_timestamp,selected_ts,angle,speed,breakpoint


def csv_to_dic(csv_f , idx):

    with open(csv_f, 'rb') as csv_file:
         lable_reader = islice(csv.reader(csv_file,delimiter=','), idx , None)
         dic = {}
         for row in lable_reader:
             dic[row[0]] = [row[1],row[3]]

    return dic


#returns dictionary with picture idetifier (time of shot) with nearest measurment (timestamp) of steering angle
def parse_center_to_angle():

    print "Starting Step  parse_center_to_angle"
    pictures = [name for name in os.listdir(center_cam) if os.path.isfile(os.path.join(center_cam, name))]
    pictures.sort()
    pictures = pictures[start:ende]
    dic = {}
    i = 0

    print len(pictures)

    csv_dic = csv_to_dic(lable_csv,1)
    csv_list = csv_dic.keys()
    csv_list.sort()

    for pic in pictures:
        pic_int = int(pic.split(".")[0])
        #epsilon default set here!!!
        pic,ts,angle,speed,index = parse_angle_picture(pic_int, 1, 100000000, csv_dic ,csv_list)


        dic[pic] = [angle,speed]
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


def create_train_test(csv_data):

    dic = load_parsed_images_from_csv(csv_data,0)

    print "test set creation"

    length = len(dic)
    length = int(length*0.25)
    center_to_hdf5("test.h5",csv_data,length,length*2)
    print "train set creation +++++++++++"

    center_to_hdf5("train.h5",csv_data,-1,-1)



def center_to_hdf5(name, csv_data,begin,end):


    dic = load_parsed_images_from_csv(csv_data,0)
    #dic = csv_to_dic(csv_data,0)
    pic_ids = dic.keys()
    pic_ids.sort()

    if(begin == -1 and end == -1):
        begin = 0
        end = len(pic_ids)

    pic_ids = pic_ids[begin:end+1]

    print len(pic_ids)

    print "Starting Step create h5py"
    train_ids = pic_ids

    with h5py.File(name, 'w') as hf:

            center_img = np.zeros([1,3,PIXEL_HEIGHT,PIXEL_WIDTH]).astype(np.float32)

            img = Image.open(center_cam+str(pic_ids[0])+".jpg").resize((PIXEL_WIDTH,PIXEL_HEIGHT),Image.NEAREST)
            center_img[0,:,:,:] = np.reshape(np.array(img),[3,PIXEL_HEIGHT,PIXEL_WIDTH])
            #center_img[0,:,:,:] = np.reshape(np.array(Image.open(center_cam+str(pic_ids[0])+".jpg")),[3,PIXEL_HEIGHT,PIXEL_WIDTH])

#            plt.imshow(img)
 #           plt.show()


            target = np.array([dic[pic_ids[0]]]).astype(np.float32)

            print target
            print "----------------"

            hf.create_dataset('data', data=center_img, maxshape=(None,3,PIXEL_HEIGHT,PIXEL_WIDTH))
            hf.create_dataset('target_label',data=target , maxshape=(None,))

            for i,img in enumerate(train_ids):
                if i != 0 and i % 3 == 0:
                    print "loading img :", i , "/" , len(pic_ids)-1

                    center_img = np.zeros([1,3,PIXEL_HEIGHT,PIXEL_WIDTH]).astype(np.float32)
                    img = Image.open(center_cam+str(pic_ids[i])+".jpg").resize((PIXEL_WIDTH,PIXEL_HEIGHT),Image.NEAREST)
                    center_img[0,:,:,:] = np.reshape(np.array(img),[3,PIXEL_HEIGHT,PIXEL_WIDTH])

                   # center_img[0,:,:,:] = np.reshape(np.array(Image.open(center_cam+str(pic_ids[i])+".jpg")),[3,PIXEL_HEIGHT,PIXEL_WIDTH])

                    next_target = np.array([dic[pic_ids[i]]]).astype(np.float32)

                    print pic_ids[i], " angle: ", next_target


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
def load_parsed_images_from_csv(v_file,with_speed):
    with open(v_file, 'rb') as csv_file:
        lable_reader = islice(csv.reader(csv_file,delimiter=','), 0, None)
        dic = {}
        for row in lable_reader:
            if with_speed == 1:
                s1 = row[1].translate(None, '[]\'')
                s2 = row[2].translate(None, '[]\'')
                dic[row[0]] = [s1,s2]
            else:
               # print row
                dic[row[0]] = row[1].translate(None,'[]\'')
    return dic


def build_slope():

    dic = load_parsed_images_from_csv("center_to_angle.csv",1)
    ordered_list = dic.keys()
    ordered_list.sort()

    save_dic = {}

    for idx, key  in enumerate(ordered_list):
        if(idx >= 5 and idx <= len(ordered_list)-5):
            print "sliding windowd from ", idx -5 , "to : " , idx+5
            x = range(0,10)
            keys_for_regression = ordered_list[idx -5:idx + 5]
            y = np.array([dic[n][0] for n in keys_for_regression]).astype(np.float)
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

    dic_angle = load_parsed_images_from_csv("center_to_angle.csv",1)
    dic_slope = load_parsed_images_from_csv("center_to_slope.csv",0)

    keys = dic_slope.keys()
    keys.sort()

    return_dic = {}
    straight_driving = {}

    for key in keys:
        angle = float(dic_angle[key][0])
        speed = float(dic_angle[key][1])
        slope = float(dic_slope[key])


        if(abs(slope) >= 0.01 or abs(angle) >= 0.5):
            if(speed > 0):
                return_dic[key] = [angle,speed]
                print "++++++++++++"
                print "accepted " , key
                print 'angle = ' , angle
                print 'speed = ' , speed
                print 'slope ='  , slope
            else:
                print "rejected because speed = " , speed

        else:
            if(speed > 0):
                straight_driving[key] = [angle,speed]
            print "rejected because slope = " ,slope
    keys = return_dic.keys()
    keys.sort()

    with open('selected_subset.csv', 'w') as f:
        for key in keys:
            f.write("%s,%s\n"%(key,return_dic[key]))

    keys = straight_driving.keys()
    keys.sort()
    with open('selected_subset_straight.csv', 'w') as f:
        for key in keys:
            f.write("%s,%s\n"%(key,straight_driving[key]))


#percent of dataset beeing straight driving
# assumption1 : there are more straight pictures than curves
# assumption2 : p_straight is not larger than 0.5
def combine_straight_curves_driving(new_name,csv_straight,csv_curves,p_straight):

    dic_straight = load_parsed_images_from_csv(csv_straight,0)
    dic_curves   = load_parsed_images_from_csv(csv_curves,0)

    a = len(dic_curves)
    size_data = int(a/(1-p_straight))
    b = int(size_data*p_straight)

    selected_subset_straight = []
    keys = dic_straight.keys()
    keys.sort()

    print 'selecting ' ,a,' curve images'
    print 'selecting ' ,b, ' straight images'
    print 'staring random select +++++'

    #randomly select straight images might be double
    for i in range(b):
        found = 0
        while(found == 0):
            rand = random.randint(0,len(keys)-1)
            key = keys[rand]

            if(key in dic_curves.keys()):
                print "resample key"
            else:
                dic_curves[key] = dic_straight[key]
                found = 1
                print "selecting ", keys[rand] ,' index: ', i

    print 'size of dic_curves ' , len(dic_curves)

    keys = dic_curves.keys()
    keys.sort()
    with open(new_name, 'w') as f:
        for key in keys:
            f.write("%s,%s\n"%(key,dic_curves[key]))



def changed_angles_in_csv(new_name,csv_file):
    dic = load_parsed_images_from_csv(csv_file,0)
    keys = dic.keys()
    keys.sort()

    out_dic = {}

    for key in keys:
       angle = dic[key]
       print "key: ", key , " angle: ",angle

       angle = float(angle)
     #  if(angle == 0):
      #     angle = 0.001

     #changed_angle = ((angle*180)/math.pi)+360
       changed_angle = angle + 2

       #changed_angle = 1/angle
       out_dic[key] = changed_angle
       print "changed to : ", changed_angle
       print "-------------------"

    keys = out_dic.keys()
    keys.sort()
    with open(new_name, 'w') as f:
        for key in keys:
            f.write("%s,%s\n"%(key,out_dic[key]))


if __name__== "__main__":

    #parse_center_to_angle()
    #parse_center_to_angle()
    #center_to_hdf5("train.h5" ,"center_to_angle.csv",-1,-1)

    s = -1
    e = -1

    if args.n:
        if args.n > 0:
            s = 0
            e = args.n

    if args.a:
        parse_center_to_angle()
        center_to_hdf5("train.h5" ,"center_to_angle.csv",s,e)

    elif args.c:
        parse_center_to_angle()
        build_slope()
        select_subset_of_turns()
       # center_to_hdf5("test.h5" ,"selected_subset.csv",start,end)
        center_to_hdf5("train.h5" ,"selected_subset.csv",s,e)

    elif args.l:
        parse_center_to_angle()
        build_slope()
        select_subset_of_turns()
        #center_to_hdf5("test.h5" ,"combined_0.1.csv",-1,-1)
        center_to_hdf5("train.h5" ,"selected_subset_straight.csv",s,e)

    elif args.s:
        p = args.s
        if p >0 and p <= 0.5:
            print p
            parse_center_to_angle()
            build_slope()
            select_subset_of_turns()
            combine_straight_curves_driving("combined_p.csv","selected_subset_straight.csv","selected_subset.csv",p)
            center_to_hdf5("train.h5" ,"combined_p.csv",s,e)
        else:
            print "Nothing done"
            print "p > 0 and p<=0.5 ?"

    elif args.r:
        string = args.r
        PIXEL_WIDTH  = int(640)
        PIXEL_HEIGHT = int(480)
        center_to_hdf5("train.h5" ,"combined_p.csv",s,e)

    else:
        print "Nothing done"
        print "see flags for more info"



#also suported
#changed_angles_in_csv("changed_angles.csv","center_to_angle.csv")
#center_to_hdf5("train.h5","changed_angles.csv",-1,-1)


