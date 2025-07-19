# (c) https://github.com/MontiCore/monticore  
import rospy
import numpy as np
import time
import sys

from std_msgs.msg import Float32MultiArray, Bool

if __name__ == '__main__':
    pubreset = rospy.Publisher('/torcs/reset', Bool, queue_size=1)
    pubstep = rospy.Publisher('/torcs/step', Float32MultiArray, queue_size=1)
    rospy.init_node('test', anonymous=True)

    for episode in range(10):
        pubreset.publish(Bool(True))
        time.sleep(5)
        for step in range(50):
            pubstep.publish(Float32MultiArray(data=np.array([0.0, 1.0, 0.0])))
            time.sleep(0.2)
            if rospy.is_shutdown():
                print('shutdown')
                sys.exit(1)

    print('FINISHED!')
