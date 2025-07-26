import rospy
from std_msgs.msg import String, UInt16MultiArray, Float32MultiArray, UInt8MultiArray
import numpy as np
from PIL import Image
import cv2

def talker():
    pub = rospy.Publisher('/torcs/vision', UInt8MultiArray, queue_size=1)
    rospy.init_node('talker', anonymous=False)
    rate = rospy.Rate(5) 

    
    while not rospy.is_shutdown():
        img = Image.open("aux/debugging-dataset/random-img.png")
        
        img = np.array(img)
        img = np.moveaxis(img, [2], [0])
        img = img.astype('uint8')
        img = img.tobytes()
        pub.publish(UInt8MultiArray(data=img))
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass