#!/usr/bin/env python

import rospy
import time
import numpy as np
import math

from std_msgs.msg import Float32MultiArray, Bool, Int32, Float32
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from movementMaintainer import *


class RosConnector(object):
    laser_topic = '/gazebo/scan'
    terminate_topic = '/gazebo/terminal'
    step_topic = '/gazebo/step'
    reset_topic = '/gazebo/reset'
    reward_topic = '/gazebo/reward'
    position_topic = '/gazebo/position'
    ros_update_rate = 10
    
    RANDOM_START_POS = False
    X_INIT = -0.4
    Y_INIT = -0.4
    THETA_INIT = 45.0
    
    X_GOAL = 2
    Y_GOAL = -1
    THETA_GOAL = -30
    
    
    def __init__(self, env_str, verbose=True):
        # initialize the node
        rospy.init_node(env_str, anonymous=True)
        
        self.__terminated = True
        self.__verbose = verbose
        self.__turtleBot_in_position = False
        self.__crash = False

        self.__goal_distance = 0.0
        self.__heading = 0 #you can replace the heading with self.__heading
        
        self.__laser_publisher = rospy.Publisher(
            RosConnector.laser_topic, Float32MultiArray,queue_size=1)
        self.__terminate_publisher = rospy.Publisher(
            RosConnector.terminate_topic, Bool, queue_size=1)
        self.__reward_publisher = rospy.Publisher(
            RosConnector.reward_topic, Float32, queue_size=1)
        self.__position_publisher = rospy.Publisher(
            RosConnector.position_topic, Float32MultiArray, queue_size=1) # publisher for the position, heading, current distance
        
        self.__set_position_publisher = rospy.Publisher(
            'gazebo/set_model_state', ModelState, queue_size=10)
        self.__set_navigation_publisher = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=10)
        
        self.__action_subscriber = rospy.Subscriber(
            RosConnector.step_topic, Int32, self.step)
        self.__reset_subscriber = rospy.Subscriber(
            RosConnector.reset_topic, Bool, self.reset)
        
        rate = rospy.Rate(10)
        rospy.spin()
        time.sleep(1) # check if with less time to sleep it would be faster inialization
        self.print_if_verbose('ROS node initialized')


    def reset(self, msg=Bool(data=True)):
        rospy.loginfo('reset') # only for logging needed to be delted
        
        if msg.data is True and self.is_terminated:
            self.__in_reset = True
            if not self.__turtleBot_in_position:
                self.positionResetter()
            self.print_if_verbose('TurtleBot started')
            time.sleep(0.1)
            
            ranges_array = Float32MultiArray()
            ranges_array.data = rospy.wait_for_message('/scan', LaserScan).ranges 
            
            odomMsg_array = Float32MultiArray()
            odomMsg = rospy.wait_for_message('/odom', Odometry)
            odomMsg_array.data = getPosition(odomMsg)
            yaw = getTurtleBotRotation(odomMsg)
            heading = getHeading(odomMsg_array.data[0], self.X_GOAL, odomMsg_array.data[1], self.Y_GOAL, yaw)
            self.__goal_distance = calcDistance(odomMsg_array.data[0], self.X_GOAL, odomMsg_array.data[1], self.Y_GOAL)
            
            odomMsg_array.data += [heading , self.__goal_distance ]
            
            
            self.__terminate_publisher.publish(Bool(False))
            self.__laser_publisher.publish(ranges_array)
            self.__position_publisher.publish(odomMsg_array) # postion, heading, distance to the goal
            self.__reward_publisher.publish(Float32(0.0))

            self.__terminated = False
            self.__in_reset = False
            self.__crash = False            
            
    def step(self, msg):
        rospy.loginfo('step: '+str(msg.data)) # only for logging needed to be delted
        if not rospy.is_shutdown():
            action = msg.data

            if self.is_terminated or self.in_reset or self.is_crash: #check if we need this conditional block
                self.print_if_verbose('Discard action because turtleBot is in reset or terminated or crashed')
                stopTurtleBot(self.__set_navigation_publisher)
                self.__crash = False
                self.__turtleBot_in_position = False
                return
            else:
                if not self.__turtleBot_in_position:
                    self.positionResetter()
                    print('positionresetter step')
                else:
                    maxAngularVel = 1.5
                    angVel = ((3 - 1)/2 - action) * maxAngularVel / 2
                    velCmd = Twist()
                    velCmd.linear.x = 0.15
                    velCmd.angular.z = angVel

                    self.__set_navigation_publisher.publish(velCmd)
                    #status_info = doTurtleBotAction(self.__set_navigation_publisher, action)
                    time.sleep(0.1)
                    msgScan = rospy.wait_for_message('/scan', LaserScan)
                    ranges_array = Float32MultiArray()
                    lidarDistances = getLidarDist(msgScan)
                    ranges = msgScan.ranges
                    ranges_array.data = ranges
                     
                    self.__crash = checkCrash(lidarDistances)
                    
                    odomMsg_array = Float32MultiArray()
                    odomMsg = rospy.wait_for_message('/odom', Odometry)
                    array_position = getPosition(odomMsg) # du kannst odomMsg_array direkt zuweisen
                    odomMsg_array.data= array_position
                    
                    yaw = getTurtleBotRotation(odomMsg)
                    heading = getHeading(odomMsg_array.data[0], self.X_GOAL, odomMsg_array.data[1], self.Y_GOAL, yaw)
                    current_distance = calcDistance(odomMsg_array.data[0], self.X_GOAL, odomMsg_array.data[1], self.Y_GOAL)
                    obstacle_min_range = round(min(ranges), 2)
                    
                    odomMsg_array.data += [heading , self.__goal_distance ]
                    
                    (reward, terminal ) = getReward(action, heading, current_distance, self.__goal_distance,obstacle_min_range, self.__crash)   
                    
                    time.sleep(0.1)
                    if terminal:
                        self.__terminated = True
                        self.__turtleBot_in_position = False
                    
                    self.__laser_publisher.publish(ranges_array)
                    self.__position_publisher.publish(odomMsg_array)
                    self.__terminate_publisher.publish(Bool(terminal))
                    self.__reward_publisher.publish(Float32(reward))


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
        #self.__first_action_taken = False
        
        if self.RANDOM_START_POS:
            (x_init, y_init, theta_init) = setTurtleBotRandomPos(
                self.__set_position_publisher)
        else:
            (x_init, y_init, theta_init) = setTurtleBotPos(
                self.__set_position_publisher, 
                self.X_INIT, self.Y_INIT, self.THETA_INIT)
            odomMsg = rospy.wait_for_message('/odom', Odometry)
            array_pos = getPosition(odomMsg)
            theta = degrees(getTurtleBotRotation(odomMsg))
            if checkDiff(array_pos[0],x_init, 0.01) and checkDiff(array_pos[1],y_init,0.01) \
                and checkDiff(theta, theta_init, 1):
                    self.__turtleBot_in_position = True
                    print("positonResetter: turlte in position")
            else:
                self.__turtleBot_in_position = False
                print("positonResetter: turlte in Not position")
    
    


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