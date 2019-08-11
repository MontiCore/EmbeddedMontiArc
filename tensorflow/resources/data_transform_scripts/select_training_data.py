import csv
from itertools import islice
import matplotlib.pyplot as plt
import numpy as np

Data_Dir = '/media/felixh/hdd/extracted_dataset.bag/'
center_cam = Data_Dir + 'center/'
left_cam =   Data_Dir + 'left/'
right_cam =  Data_Dir + 'right/import matplotlib.pyplot as plt'
lable_csv = Data_Dir + 'steering.csv'




def select_subset():
    with open(lable_csv, 'rb') as csv_file:
        lable_reader = islice(csv.reader(csv_file,delimiter=','), 1, None)

        ys = []
        xs = []

        i = 1
        for i,row in lable_reader:
            print i
            angle = row[1]
            angle_timestamp = row[0]



if __name__== "__main__":
    select_subset()
