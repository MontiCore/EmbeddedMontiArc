from h5py._hl import dataset
import rospy
from std_msgs.msg import String, UInt16MultiArray, Float32MultiArray, UInt8MultiArray, Bool
import numpy as np
from PIL import Image
import cv2
import h5py
import time

def talker():
    camera_pub = rospy.Publisher('/torcs/vision', UInt8MultiArray, queue_size=1)
    affordance_pub = rospy.Publisher('/torcs/groundtruth', Float32MultiArray, queue_size=1)
    reset_pub = rospy.Publisher('/torcs/reset', Bool, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) 
    dataset = h5py.File("training-data-debugging-small/train.h5", "r")
    cnt = 0
    reset_pub.publish(Bool(True))
    time.sleep(1)
    reset_pub.publish(Bool(False))

    while not rospy.is_shutdown():
        img = dataset["data"][cnt]
        affordance = dataset["predictions_label"][cnt]
        img = img.astype('uint8')
        img = img.tobytes()
        camera_pub.publish(UInt8MultiArray(data=img))
        affordance_pub.publish(Float32MultiArray(data=affordance))
        cnt += 1
        rate.sleep()
        if cnt == 799:
            cnt = 0

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass