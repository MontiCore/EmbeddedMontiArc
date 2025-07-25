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

    print("------------------")
    print("always-on affordance")
    print("%s : %.3f \t (%.2f rad = %.2f deg)" % (label_names[0], label_value[0],
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
        print("track width: %.2f" %
              (denormalized_label_values[3] - denormalized_label_values[1]))
    print("------------------")

    print("in-lane affordance:")
    for name, value, denormalized_value in zip(label_names[6: -1], label_value[6: -1], denormalized_label_values[6: -1]):
        print("%s : %.2f \t (%.2f m)" % (name, value, denormalized_value))

    distLL = label_value[6]
    distRR = label_value[9]
    angle = denormalized_label_values[0]
    speed = 1

    if (distLL > 0 and distRR < 1):
        track_width = denormalized_label_values[9] - \
            denormalized_label_values[6]
        track_pos = (track_width - denormalized_label_values[9]) / track_width
        score = (speed * np.cos(angle)) - \
            (speed * np.sin(angle)) - (speed * track_pos)
        print("\ntrack width: %.2f" % (track_width))
        print("track pos: %.2f" % (track_pos))
        print("reward: %.2f" % (score))
    elif(distLL > 0 and distRR == 1):
        track_width = denormalized_label_values[8] - \
            denormalized_label_values[6]
        track_pos = (track_width - denormalized_label_values[8]) / track_width
        score = (speed * np.cos(angle)) - \
            (speed * np.sin(angle)) - (speed * track_pos)
        print("\ntrack width: %.2f" % (track_width))
        print("track pos: %.2f" % (track_pos))
        print("reward: %.2f" % (score))
    elif(distLL == 0 and distRR < 1):
        track_width = denormalized_label_values[9] - \
            denormalized_label_values[7]
        track_pos = (track_width - denormalized_label_values[9]) / track_width
        score = (speed * np.cos(angle)) - \
            (speed * np.sin(angle)) - (speed * track_pos)
        print("\ntrack width: %.2f" % (track_width))
        print("track pos: %.2f" % (track_pos))
        print("reward: %.2f" % (score))
    print("------------------")


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

    #print_label(data.data, label_range)
    #print(np.array(data.data).shape)
    print("angle: %.3f degree" % (data.data[0]))


def listener():
    rospy.init_node('affordance_listener', anonymous=False)

    rospy.Subscriber('/torcs/affordance', Float32MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
