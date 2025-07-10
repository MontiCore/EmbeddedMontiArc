# (c) https://github.com/MontiCore/monticore  
#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray, Int32

rospy.init_node('converter', anonymous=True) 
trajectory_x_pub = rospy.Publisher('autopilot/trajectory_x', Float64MultiArray)
trajectory_y_pub = rospy.Publisher('autopilot/trajectory_y', Float64MultiArray)
length_pub = rospy.Publisher('autopilot/trajectory_length', Int32)
rate = rospy.Rate(10) # 10hz

def callback(path):
   
    trajectory_x = Float64MultiArray()
    trajectory_y = Float64MultiArray()  
    trajectory_length = min(100, len(path.poses))

    del trajectory_x.data[:]
    del trajectory_y.data[:]

    for i in range(0, (trajectory_length)):
        (trajectory_x.data).append(round((((path.poses)[i]).pose.position.x),2))
        (trajectory_y.data).append(round((((path.poses)[i]).pose.position.y),2))
    if (trajectory_length < 100):
        for j in range(trajectory_length, 100):
            (trajectory_x.data).append(0.00)
            (trajectory_y.data).append(0.00)

    #rospy.loginfo(trajectory_length)
    length_pub.publish(trajectory_length)

    #rospy.loginfo(trajectory_x)
    trajectory_x_pub.publish(trajectory_x)

    #rospy.loginfo(trajectory_y)
    trajectory_y_pub.publish(trajectory_y)



def converter(): 

    rospy.Subscriber("/carla/ego_vehicle/waypoints", Path, callback)

    #rospy.spin()

    

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
             converter()
             rate.sleep()
    except rospy.ROSInterruptException:
        pass
