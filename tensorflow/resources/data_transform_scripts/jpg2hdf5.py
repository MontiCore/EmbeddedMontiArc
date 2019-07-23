import os, os.path
import glob
import csv
from itertools import islice
import h5py
import numpy as np
from PIL import Image

#for1475186966202305668 resizing
import cv2

#DIR Dictionary setup
Data_Dir = '/media/felixh/data_ssd/test_set/'
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

len_pictures = len([name for name in os.listdir(left_cam) if os.path.isfile(os.path.join(left_cam, name))])
print "Number pictures to format: left cam",len_pictures

len_pictures = len([name for name in os.listdir(right_cam) if os.path.isfile(os.path.join(right_cam, name))])
print "Number pictures to format: right cam",len_pictures

print "---------------------------"

#UNUSED right now
#might be helpfull later
# https://www.tutorialkart.com/opencv/python/opencv-python-resize-image/i
# checkout why resizing might be usefull https://towardsdatascience.com/boost-your-cnn-image-classifier-performance-with-progressive-resizing-in-keras-a7d96da06e20
def rezize_img(img):

    img = cv2.imread('/media/felixh/data_ssd/test_set/1475186965652132892.jpg', cv2.IMREAD_UNCHANGED)
    print('Original Dimensions : ',img.shape)

    scale_percent = 50 # percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    # resize image
    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

    print('Resized Dimensions : ',resized.shape)

    #cv2.imshow("Resized image", resized)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

    return resized



# helper function to acomplish parse_all_pictures
#increase
def parse_angle_picture(pic_timestamp, epsilon):
    pic_timestamp = int(pic_timestamp)
    with open(lable_csv, 'rb') as csv_file:
        lable_reader = islice(csv.reader(csv_file,delimiter=','), 1, None)
        distance = epsilon
        selected_row = None
        selected_ts = 0
        for row in lable_reader:
            ts_int = int(row[0])
            diff = pic_timestamp-ts_int
            if(diff > 0  - epsilon):
                if(abs(diff) < distance):
                    distance = abs(diff)
                    del row[0]
                    selected_row = row
                    selected_ts = ts_int
            else:
                break

    if(selected_row == None):
        raise Exception("No suitable match found: try increase epsilon")

    return pic_timestamp, selected_row, distance


#returns dictionary with picture idetifier (time of shot) with nearest measurment (timestamp) of steering angle
def parse_center_to_angle():
    print "Starting Step  parse_center_to_angle"
    pictures = [name for name in os.listdir(center_cam) if os.path.isfile(os.path.join(center_cam, name))]
    dic = {}
    i = 0
    for pic in pictures:
        pic_int = int(pic.split(".")[0])
        #epsilon default set here!!!
        pic,row,dis = parse_angle_picture(pic_int, 100000000)
        dic[pic] = row
        i = i + 1
        if i % 10 == 0:
            print "proccesed pictures :" , i , "/", len(pictures)

    return dic

# tensorflow seem to have height width channels not vise versa
# https://www.kaggle.com/crawford/resize-and-save-images-as-hdf5-256x256
# why images in 256 * 256 ????




# return dictionary with center_cam pic as key which is associated with list  [left_img,right_img,steeringangle,N.A,speed]
def parse_center_to_left_right():

    print "STARTING Step  parse center to left"
    # bla bla
    dic = parse_center_to_angle()
    pic_ids = dic.keys()
    lable_csv = Data_Dir + 'steering.csv'
    pic_ids.sort()

    right_cam_list = [int(name.split(".")[0]) for name in os.listdir(right_cam) if os.path.isfile(os.path.join(right_cam, name))]
    left_cam_list = [int(name.split(".")[0]) for name in os.listdir(left_cam) if os.path.isfile(os.path.join(left_cam, name))]

    #sort for matching
    right_cam_list.sort()
    left_cam_list.sort()

    #consitency check leave out in actuall computation to save time
    i = 0
    epsilon = 100000000
    for pic in pic_ids:

        if i % 10 == 0:
            print "matched pictures :" , i , "/", len(pic_ids)

        diff = pic - right_cam_list[i]
        diff2 = pic -left_cam_list[i]

        dic[pic].insert(0,right_cam_list[i])
        dic[pic].insert(0,left_cam_list[i])

        if diff > epsilon or diff2 > epsilon:
             raise Exception("do consitency check in parsing center to left right cam: images matched to distant from eachother")
        i = i+1

    return dic

def create_h5py(train_percentage, test_percentage):

    dic = parse_center_to_left_right()
    #sorted list based on timestamps
    pic_ids = dic.keys()
    pic_ids.sort()

    #reloading glob ids for images

    #print ("reloading images with glob")

    print "Starting Step create h5py"
    if(train_percentage + test_percentage == 1):
        train_ids = pic_ids[0:int(train_percentage*len(pic_ids))]
       # test_ids =  pic_ids[int(test_percentage*len(pic_ids)):]
     #   print "length train ids:",len(train_ids)
       # print "length test ids:",len(test_ids)

        # https://stackoverflow.com/questions/47072859/how-to-append-data-to-one-specific-dataset-in-a-hdf5-file-with-h5py
        with h5py.File('train.h5', 'w') as hf:

            #create starting
            left_img = np.zeros([1,3,PIXEL_HEIGHT,PIXEL_WIDTH]).astype(np.float32)
            left_img_id = dic[pic_ids[0]][0]
            left_img[0,:,:,:] = np.reshape(np.array(Image.open(left_cam+str(left_img_id)+".jpg")),[3,PIXEL_HEIGHT,PIXEL_WIDTH])

            center_img = np.zeros([1,3,PIXEL_HEIGHT,PIXEL_WIDTH]).astype(np.float32)
            center_img[0,:,:,:] = np.reshape(np.array(Image.open(center_cam+str(pic_ids[0])+".jpg")),[3,PIXEL_HEIGHT,PIXEL_WIDTH])


            right_img = np.zeros([1,3,PIXEL_HEIGHT,PIXEL_WIDTH]).astype(np.float32)
            right_img_id = dic[pic_ids[0]][1]
            right_img[0,:,:,:] = np.reshape(np.array(Image.open(right_cam+str(right_img_id)+".jpg")),[3,PIXEL_HEIGHT,PIXEL_WIDTH])


            combined_img = np.concatenate((left_img,center_img,right_img),axis=1)
      #      print combined_img.shape

            target = np.array([dic[pic_ids[0]][2]]).astype(np.float32)

            hf.create_dataset('data', data=combined_img, maxshape=(None,9,PIXEL_HEIGHT,PIXEL_WIDTH))
            hf.create_dataset('ergebnis_label',data=target , maxshape=(None,))

            for i,img in enumerate(train_ids):
                if i != 0:
                        print "loading img :", i , "/" , len(pic_ids)-1

                        left_img = np.zeros([1,3,PIXEL_HEIGHT,PIXEL_WIDTH]).astype(np.float32)
                        left_img_id = dic[pic_ids[i]][0]
                        left_img[0,:,:,:] = np.reshape(np.array(Image.open(left_cam+str(left_img_id)+".jpg")),[3,PIXEL_HEIGHT,PIXEL_WIDTH])

                        center_img = np.zeros([1,3,PIXEL_HEIGHT,PIXEL_WIDTH]).astype(np.float32)
                        center_img[0,:,:,:] = np.reshape(np.array(Image.open(center_cam+str(pic_ids[i])+".jpg")),[3,PIXEL_HEIGHT,PIXEL_WIDTH])

                        right_img = np.zeros([1,3,PIXEL_HEIGHT,PIXEL_WIDTH]).astype(np.float32)
                        right_img_id = dic[pic_ids[i]][1]
                        right_img[0,:,:,:] = np.reshape(np.array(Image.open(right_cam+str(right_img_id)+".jpg")),[3,PIXEL_HEIGHT,PIXEL_WIDTH])

                        combined_img = np.concatenate((left_img,center_img,right_img),axis=1)
                        next_target = np.array([dic[pic_ids[i]][2]]).astype(np.float32)

                        hf['data'].resize((hf['data'].shape[0] + combined_img.shape[0]), axis = 0)
                        hf['data'][-combined_img.shape[0]:] = combined_img

                        hf['ergebnis_label'].resize((hf['ergebnis_label'].shape[0] + next_target.shape[0]), axis = 0)
                        hf['ergebnis_label'][-next_target.shape[0]:] = next_target


    else:
        raise Exception("train + test != 1")





#create_h5py(0.1,0.9)

#parse_center_to_left_right()

create_h5py(1,0.0)


