from matplotlib import pyplot as plt
import rospy
from std_msgs.msg import String, UInt16MultiArray, Float32MultiArray, UInt8MultiArray
import numpy as np
import cv2


def denormalize(label, label_range):
    return np.array([denormalize_value(value, label_range[i]) for i, value in enumerate(label)])


def denormalize_value(value, label_range):
    return value * (label_range[1] - label_range[0]) + label_range[0]


def print_label(label_value, label_range):
    label_names = ["angle", "toMarking_L", "toMarking_M", "toMarking_R", "dist_L",
                   "dist_R", "toMarking_LL", "toMarking_ML", "toMarking_MR", "toMarking_RR",
                   "dist_LL", "dist_MM", "dist_RR", "fast"]
    label_units = ["rad", "m", "m", "m", "m", "m",
                   "m", "m", "m", "m", "m", "m", "m", ""]
    denormalized_label_values = denormalize(label_value, label_range)

    plt.clf()
    plt.bar(label_names, label_value)
    plt.draw()
    plt.pause(0.1)


    


def callback(data):
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

    angle = data.data[0]
    toMarking_L = data.data[1]
    toMarking_M = data.data[2]
    toMarking_R = data.data[3]
    toMarking_LL = data.data[6]
    toMarking_RR = data.data[9]

    denormalized_labels = denormalize(data.data, label_range)
    #print_label(data.data, label_range)
    #print(np.array(data.data).shape)
    #print("angle: %.3f degree, || toMarking_L %.3f || toMarkingR %.3f" % (denormalized_labels[0], denormalized_labels[1], denormalized_labels[3]))
    print("angle: %.3f degree || toMarking_L %.3f || toMarkingR %.3f || toMarking_LL %.3f || toMarking_RR %.3f" % (angle, toMarking_L, toMarking_R, toMarking_LL, toMarking_RR))

def listener():
    rospy.init_node('affordance_listener', anonymous=False)

    rospy.Subscriber('/torcs/affordance', Float32MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
