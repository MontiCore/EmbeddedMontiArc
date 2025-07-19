from matplotlib import pyplot as plt
import rospy
from std_msgs.msg import String, UInt16MultiArray, Float32MultiArray, UInt8MultiArray
import numpy as np
import cv2

cnt = 0

def callback(data):
    if False:
        img = np.frombuffer(data.data, dtype=np.uint8).reshape((3, 64, 64))
    else:
        img = np.zeros((3, 64, 64))
        cnt = 0
        for i0 in range(3):
            for i1 in range(64):
                for i2 in range(64):
                    if 0 <= cnt:
                        img[i0, i1, i2] = data.data[cnt]
                    elif 0 > cnt:
                        img[i0, i1, i2] = 0
                    else:
                        img[i0, i1, i2] = 0
                    cnt += 1
        img = img.astype('uint8')

    img = np.moveaxis(img, [0], [2])
    if True:
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        img = cv2.resize(img, dsize=(512, 512), interpolation=cv2.INTER_CUBIC)  # or inter cubic or inter_area
        #img = cv2.flip(img, 0) #!!!!!!!! 
        cv2.imshow("camera", img)
        cv2.waitKey(10)
    else:
        plt.imshow(img, origin='lower')
        plt.draw()
        plt.pause(0.001)
    
        

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=False)

    rospy.Subscriber('/torcs/vision', UInt8MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()




 