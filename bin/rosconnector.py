#!/usr/bin/env python

from pickle import FALSE
import rospy
import time
import numpy as np

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
    
    RANDOM_START_POS = FALSE
    X_INIT = -0.4
    Y_INIT = -0.4
    THETA_INIT = 45.0
    
    current_LaserScan = LaserScan()
    
    
    def __init__(self, env_str, verbose=True, render_per_Step=1, continuous=False):
        self.__terminated = True
        self.__continuous = continuous
        self.__verbose = verbose
        self.__last_game_score = 0
        self.__turtleBot_in_position = True
        self.__first_action_taken = False
        self.__crash = 0
        self.__prev_lidar_distances = None
        self.__prev_action = None
        
        self.__laser_publisher = rospy.Publisher(
            RosConnector.laser_topic, Float32MultiArray,queue_size=1)
        self.__terminate_publisher = rospy.Publisher(
            RosConnector.terminate_topic, Bool, queue_size=1)
        self.__reward_publisher = rospy.Publisher(
            RosConnector.reward_topic, Float32, queue_size=1)
        self.__position_publisher = rospy.Publisher(
            RosConnector.position_topic, Float32MultiArray, queue_size=1)
        
        self.__set_position_publisher = rospy.Publisher(
            'gazebo/set_model_state', ModelState, queue_size=10)
        self.__set_navigation_publisher = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=10)
        
        
        self.__state_subscriber = rospy.Subscriber(
            '/scan', LaserScan, self.get_current_LaserScan)        
        self.__action_subscriber = rospy.Subscriber(
            RosConnector.step_topic, Int32, self.step)
        self.__reset_subscriber = rospy.Subscriber(
            RosConnector.reset_topic, Bool, self.reset)
        #env_str = gazeboEnv
        rospy.init_node(env_str, anonymous=True)
        rate = rospy.Rate(10)
        rospy.spin()
        time.sleep(2)
        self.print_if_verbose('ROS node initialized')

    @property
    def get_current_LaserScan(laserScan):
        global current_LaserScan
        current_LaserScan = laserScan
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
    def is_continuous(self):
        return self.__continuous
    
    @property
    def last_game_score(self):
        pass

    def reset(self, msg=Bool(data=True)):
        #msg_laserScan = LaserScan()
        #msg_laserScan.ranges = np.zeros(360)
        
        if msg.data is True and self.is_terminated:
            self.__in_reset = True
            self.__score = 0 # ToDo: need to be used
            self.__current_step = 0 # ToDo: need to be used
            #state = self.reset_turtleBot() #ToDo define this function to set the turtlebot on the starting position. Use the method from the step function.
            self.print_if_verbose('TurtleBot started')
            time.sleep(1)
            self.__terminate_publisher.publish(Bool(False))
            msgScan = rospy.wait_for_message('/scan', LaserScan)
            ranges = msgScan.ranges
            ranges_array = Float32MultiArray()
            ranges_array.data = ranges 
            self.__laser_publisher.publish(ranges_array)
            odomMsg = rospy.wait_for_message('/odom', Odometry)
            odomMsg_array = Float32MultiArray()
            odomMsg_array.data = getPosition(odomMsg)
            self.__position_publisher.publish(odomMsg_array)
            self.__reward_publisher.publish(Float32(0.0))
            self.__terminated = False
            self.__in_reset = False

    def step(self, msg):
        if not rospy.is_shutdown():
            action = msg.data
            msgScan = rospy.wait_for_message('/scan', LaserScan) #maybe should be moved under the else case

            if self.is_terminated or self.in_reset or self.is_crash:
                self.print_if_verbose('Discard action because turtleBot is in reset or terminated or crashed')
                stopTurtleBot(self.__set_navigation_publisher)
                self.__crash = 0
                self.__turtleBot_in_position = False
                self.__first_action_taken = False
                return
            else:
                if not self.__turtleBot_in_position:
                    stopTurtleBot(self.__set_navigation_publisher)
                    self.__first_action_taken = False
                    if self.RANDOM_START_POS:
                        (x_init, y_init, theta_init) = setTurtleBotRandomPos(
                            self.__set_position_publisher)
                    else:
                        (x_init, y_init, theta_init) = setTurtleBotPos(
                            self.__set_position_publisher, 
                            self.X_INIT, self.Y_INIT, self.THETA_INIT)
                        odomMsg = rospy.wait_for_message('/odom', Odometry)
                        (x , y) = getTurtleBotPos(odomMsg)
                        theta = degrees(getTurtleBotRotation(odomMsg))
                        if checkDiff(x,x_init, 0.01) and checkDiff(y,y_init,0.01) \
                            and checkDiff(theta, theta_init, 1):
                                self.__turtleBot_in_position = True
                        else:
                            self.__turtleBot_in_position = False
                elif not self.__first_action_taken:
                    lidarDistances = getLidarDist(msgScan)
                    self.__crash = checkCrash(lidarDistances) 
                    if not self.__crash:
                        status_info = doTurtleBotAction(self.__set_navigation_publisher, action)
                    self.__prev_lidar_distances = lidarDistances
                    self.__prev_action = action
                    self.__first_action_taken = True
                else:
                    lidarDistances = getLidarDist(msgScan)
                    self.__crash = checkCrash(lidarDistances) 
                    if not self.__crash:
                        status_info = doTurtleBotAction(self.__set_navigation_publisher, action)
                    (reward, terminal ) = getReward(action, self.__prev_action, lidarDistances, self.__prev_lidar_distances, self.__crash)   
                    msgScan = rospy.wait_for_message('/scan', LaserScan)
                    if terminal:
                        self.__terminated = True
                    self.__prev_lidar_distances = lidarDistances
                    self.__prev_action = action
                    ranges = msgScan.ranges
                    ranges_array = Float32MultiArray()
                    ranges_array.data = ranges 
                    self.__laser_publisher.publish(ranges_array)
                    odomMsg = rospy.wait_for_message('/odom', Odometry)
                    odomMsg_array = Float32MultiArray()
                    odomMsg_array.data = getPosition(odomMsg)
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
    