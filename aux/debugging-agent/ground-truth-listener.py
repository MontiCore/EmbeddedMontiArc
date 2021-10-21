from matplotlib import pyplot as plt
import rospy
from std_msgs.msg import String, UInt16MultiArray, Float32MultiArray, UInt8MultiArray
import numpy as np
import cv2
import time
import message_filters
from sensor_msgs.msg import Image
import sys

class Server:
    def __init__(self):
        self.label = np.zeros(14)
        self.pred = np.zeros(14)
        
        self.avg_size = 25
        self.mae_avg = np.zeros(self.avg_size)
        self.l2_avg = np.zeros(self.avg_size)
        self.mse_avg = np.zeros(self.avg_size)
        self.running_avg_cnt = 0
    
    def callback_label(self, data):
        self.label = np.array(data.data)
        
    
    def callback_pred(self, data):
        self.pred = np.array(data.data)
        self.print_error()

    def print_error(self):
        self.pred[1] = self.label[1] # was overwritten in aff-comp
        mae = sum(abs(self.pred - self.label)) / 14
        l2 = np.sqrt(sum((self.pred - self.label) ** 2))
        mse = sum((self.pred - self.label) ** 2) / 14

        self.mae_avg[self.running_avg_cnt] = mae
        self.l2_avg[self.running_avg_cnt] = l2
        self.mse_avg[self.running_avg_cnt] = mse
        self.running_avg_cnt = (self.running_avg_cnt + 1) % self.avg_size

        avg_mae = sum(self.mae_avg) / self.avg_size
        avg_l2 = sum(self.l2_avg) / self.avg_size
        avg_mse = sum(self.mse_avg) / self.avg_size

        sys.stdout.write("\rAverage Error: l2: %.3f \tmse: %3f \tmae: %.3f \t\t\t Current Error: l2: %.3f \tmse: %3f \tmae: %.3f" % (avg_l2, avg_mse, avg_mae, l2, mse, mae))
        sys.stdout.flush()

def print_label(label_value, label_range):
    label_names = ["angle", "toMarking_L", "toMarking_M", "toMarking_R", "dist_L", \
        "dist_R", "toMarking_LL", "toMarking_ML", "toMarking_MR", "toMarking_RR", \
        "dist_LL", "dist_MM", "dist_RR", "fast"]
    label_units = ["rad", "m", "m", "m", "m", "m", "m", "m", "m", "m", "m", "m", "m", ""]
    denormalized_label_values = denormalize(label_value, label_range)
    
    print("------------------")
    print("always-on affordance")
    print("%s : %.3f \t (%.2f rad = %.2f deg)" % (label_names[0], label_value[0], \
        denormalized_label_values[0], denormalized_label_values[0] / np.pi * 180))
    print(label_names[-1] + ": " + str(label_value[-1]))
    print("------------------")

    print("on-mark affordance:")
    for name, value, denormalized_value in zip(label_names[1:6], label_value[1:6], denormalized_label_values[1:6]):
        print("%s : %.2f \t (%.2f m)" % (name, value, denormalized_value))
        #print(name + ": " + str(value))
    distL = label_value[1]
    distR = label_value[3]
    if(distL > 0 and distR < 1):
        print("track width: %.2f" % (denormalized_label_values[3] - denormalized_label_values[1]))
    print("------------------")

    print("in-lane affordance:")
    for name, value, denormalized_value in zip(label_names[6: -1], label_value[6: -1], denormalized_label_values[6: -1]):
        print("%s : %.2f \t (%.2f m)" % (name, value, denormalized_value))

    distLL = label_value[6]
    distRR = label_value[9]
    angle = denormalized_label_values[0]
    speed = 1

    if (distLL > 0 and distRR < 1):
        track_width = denormalized_label_values[9] - denormalized_label_values[6]
        track_pos = (track_width - denormalized_label_values[9]) / track_width
        score = (speed * np.cos(angle)) - (speed * np.sin(angle)) - (speed * track_pos)
        print("\ntrack width: %.2f" % (track_width))
        print("track pos: %.2f" %(track_pos))
        print("reward: %.2f" %(score))
    elif(distLL > 0 and distRR == 1):
        track_width = denormalized_label_values[8] - denormalized_label_values[6]
        track_pos = (track_width - denormalized_label_values[8]) / track_width
        score = (speed * np.cos(angle)) - (speed * np.sin(angle)) - (speed * track_pos)
        print("\ntrack width: %.2f" % (track_width))
        print("track pos: %.2f" %(track_pos))
        print("reward: %.2f" %(score))
    elif(distLL == 0 and distRR < 1):
        track_width = denormalized_label_values[9] - denormalized_label_values[7]
        track_pos = (track_width - denormalized_label_values[9]) / track_width
        score = (speed * np.cos(angle)) - (speed * np.sin(angle)) - (speed * track_pos)
        print("\ntrack width: %.2f" % (track_width))
        print("track pos: %.2f" %(track_pos))
        print("reward: %.2f" %(score))
    print("------------------")


def denormalize(label, label_range):
    return np.array([denormalize_value(value, label_range[i]) for i, value in enumerate(label)])


def denormalize_value(value, label_range):
    return value * (label_range[1] - label_range[0]) + label_range[0]


def tmp(data):
    angle_range = [-2.5, 2.5]
    toMarking_L_range = [-7, 0]
    toMarking_M_range = [-2, 3.5]
    toMarking_R_range = [0, 7]
    dist_L_range = [0, 75]
    dist_R_range = [0, 75]
    toMarking_LL_range = [-9.5, 0]
    toMarking_ML_range = [-5.5, 0]
    toMarking_MR_range = [0, 5.5]
    toMarking_RR_range = [0, 9.5]
    dist_LL_range = [0, 75]
    dist_MM_range = [0, 75]
    dist_RR_range = [0, 75]
    fast_range = [0, 1]

    label_range = [angle_range, toMarking_L_range, toMarking_M_range, \
        toMarking_R_range, dist_L_range, dist_R_range, toMarking_LL_range, \
        toMarking_ML_range, toMarking_MR_range, toMarking_RR_range, \
        dist_LL_range, dist_MM_range, dist_RR_range, fast_range]
    #print_label(data.data, label_range)
    np.set_printoptions(precision=3)
    print(np.array(data.data[:7]))


def listener():
    rospy.init_node('affordance_listener', anonymous=False)

    server = Server()
    label_sub = rospy.Subscriber('/torcs/groundtruth', Float32MultiArray, server.callback_label, queue_size=1)
    pred_sub = rospy.Subscriber('/torcs/affordance', Float32MultiArray, server.callback_pred
    , queue_size=1)

    #ts = message_filters.TimeSynchronizer([label_sub, pred_sub], 10)
    #ts = message_filters.TimeSynchronizer([label_sub, pred_sub], 10)
    #ts.registerCallback(callback2)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
