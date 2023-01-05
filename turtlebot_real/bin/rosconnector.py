#!/usr/bin/env python

import rospy
import time
import numpy as np
import math
import thread

from std_msgs.msg import Float32MultiArray, Bool, Int32, Float32
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from movementMaintainer import *


class RosConnector(object):
    laser_topic = '/gazebo/scan'
    step_topic = '/gazebo/step'
    reset_topic = '/gazebo/reset'
    goalReachedBool_topic = '/gazebo/goalReachedBool'
    position_topic = '/gazebo/position'
    actionIn_topic = '/gazebo/actionIn'
    resetState_topic = '/gazebo/resetState'
    
    goalReached_topic = '/gazebo/goalReached'
    ros_update_rate = 10
    
    RANDOM_START_POS = False
    X_INIT = -2.0
    Y_INIT = -0.5
    THETA_INIT = 0.0
    
    def __init__(self, env_str, verbose=True):
        # initialize the node
        print('Start init:')
        rospy.init_node(env_str, anonymous=True)
        
        self.__terminated = True
        self.__verbose = verbose
        self.__turtleBot_in_position = False
        self.__crash = False
        self.__goal_reached = False

        self.__goal_distance = 0.0
        self.__heading = 0 #you can replace the heading with self.__heading
        
        self.__laser_publisher = rospy.Publisher(
            RosConnector.laser_topic, Float32MultiArray,queue_size=1)
        self.__position_publisher = rospy.Publisher(
            RosConnector.position_topic, Float32MultiArray, queue_size=1)
        
        self.__actionIn_publisher = rospy.Publisher(
            RosConnector.actionIn_topic, Int32, queue_size=1)
        
        self.__resetState_publisher = rospy.Publisher(
            RosConnector.resetState_topic, Bool, queue_size=1)
        
        self.__goalReachedBool_publisher = rospy.Publisher(
            RosConnector.goalReachedBool_topic, Bool, queue_size=1)
        
        self.__set_position_publisher = rospy.Publisher(
            'gazebo/set_model_state', ModelState, queue_size=10)
        self.__set_navigation_publisher = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=10)
        
        self.__goalReached_subscriber = rospy.Subscriber(
            RosConnector.goalReached_topic, Bool, self.goalReached)
        self.__action_subscriber = rospy.Subscriber(
            RosConnector.step_topic, Float32MultiArray, self.step)
        self.__reset_subscriber = rospy.Subscriber(
            RosConnector.reset_topic, Bool, self.reset)
        
        
        rate = rospy.Rate(10)
        #rospy.spin()
        self__listener = thread.start_new_thread(rospy.spin, ())
        time.sleep(1) # check if with less time to sleep it would be faster inialization
        self.print_if_verbose('ROS node initialized')
        

    def reset(self, msg=Bool(data=True)):
        rospy.loginfo('reset') # only for logging needed to be delted
        
        if msg.data is True:
            self.__in_reset = True
            self.__turtleBot_in_position = False # added only for realbot 07.12.
            if not self.__turtleBot_in_position:
                self.positionResetter()

            msgScan = None
            while msgScan is None:
                try:
                    msgScan = rospy.wait_for_message('/scan', LaserScan, timeout=5)
                except:
                    pass

            ranges_array = Float32MultiArray()
            ranges = msgScan.ranges
            ranges_array.data = ranges
            
            odomMsg_array = Float32MultiArray()
            
            odomMsg = None
            while odomMsg is None:
                try:
                    odomMsg = rospy.wait_for_message('/odom', Odometry, timeout=5)
                except:
                    pass            
            
            odomMsg_array.data = getPosition(odomMsg)
                        
            orientation_q = odomMsg.pose.pose.orientation
            odomMsg_array.data += [ orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]
            
            self.__actionIn_publisher.publish(Int32(10))
            self.__resetState_publisher.publish(Bool(True))
            
            self.__laser_publisher.publish(ranges_array)
            self.__position_publisher.publish(odomMsg_array)
            self.__goalReachedBool_publisher.publish(Bool(self.__goal_reached))

            self.__terminated = False
            self.__in_reset = False
            self.__crash = False
            self.__goal_reached = False        
            
    def step(self, msg):
        if not rospy.is_shutdown():
            
            cmd_vel_data = msg.data
            action = cmd_vel_data[0]
            cmd_vel_linear = cmd_vel_data[1]
            cmd_vel_angular = cmd_vel_data[2]

            if self.is_terminated or self.in_reset or self.is_crash: #check if we need this conditional block
                rospy.loginfo('Discard action because turtleBot is in reset or terminated or crashed')
                stopTurtleBot(self.__set_navigation_publisher)
                self.__crash = False
                self.__turtleBot_in_position = False
                return
            else:
                if not self.__turtleBot_in_position:
                    self.positionResetter()
                else:
                    status_info = doTurtleBotAction(self.__set_navigation_publisher, cmd_vel_linear, cmd_vel_angular)
                    
                    msgScan = None
                    while msgScan is None:
                        try:
                            msgScan = rospy.wait_for_message('/scan', LaserScan, timeout=5)
                        except:
                            pass

                    ranges_array = Float32MultiArray()
                    ranges_array.data = msgScan.ranges
                    
                    self.__crash = False
                    odomMsg_array = Float32MultiArray()
                    
                    odomMsg = None
                    while odomMsg is None:
                        try:
                            odomMsg = rospy.wait_for_message('/odom', Odometry, timeout=5)
                        except:
                            pass

                    array_position = getPosition(odomMsg) # du kannst odomMsg_array direkt zuweisen
                    odomMsg_array.data= array_position
                                        
                    orientation_q = odomMsg.pose.pose.orientation
                    odomMsg_array.data += [ orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]
                    
                    self.__goalReachedBool_publisher.publish(Bool(self.__goal_reached))
                    self.__actionIn_publisher.publish(Int32(action))
                    self.__resetState_publisher.publish(Bool(False))
                    self.__laser_publisher.publish(ranges_array)
                    self.__position_publisher.publish(odomMsg_array)
                    
    def goalReached(self, msg=Bool(data=True)):
        if msg.data is True:
            self.__goal_reached = True
        else:
            self.__goal_reached = False


    def print_if_verbose(self, massage):
        if self.__verbose:
            rospy.loginfo(massage)

    def shutdown(self):
        rospy.signal_shutdown('Shutdown')
        
    def createActions():
        actions = np.array([0,1,2])
        return actions
    
    def positionResetter(self):
        stopTurtleBot(self.__set_navigation_publisher)
        
        (x_init, y_init, theta_init) = setTurtleBotPos(self.__set_position_publisher,self.X_INIT, self.Y_INIT, self.THETA_INIT)
        odomMsg = None
        while odomMsg is None:
            try:
                odomMsg = rospy.wait_for_message('/odom', Odometry, timeout=5)
            except:
                pass

        array_pos = getPosition(odomMsg)
        if abs(array_pos[0]- x_init) <  0.01 and abs(array_pos[1] - y_init) < 0.01:
            self.__turtleBot_in_position = True
        else:
            self.__turtleBot_in_position = False
    
    


    @property
    def is_terminated(self):
        return self.__terminated
    
    @property
    def is_crash(self):
        return self.__crash
    
    @property
    def in_reset(self):
        return self.__in_reset
    
    @property
    def last_game_score(self):
        pass