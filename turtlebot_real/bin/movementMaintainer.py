#!/usr/bin/env python

import rospy
import math
import numpy as np
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from math import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def stopTurtleBot(velPub):
    velMsg = createCmdVelMsg(0.0,0.0)
    velPub.publish(velMsg)
    
def createCmdVelMsg(v,w):
    velMsg = Twist()
    velMsg.linear.x = v
    velMsg.angular.z = w
    return velMsg
    
def setRandomGoalPos():

    #Stage 1
    goal_x_list = [0.9, 1.5, 1.5, 0.2, -1.0, -1.5, -0.3]
    goal_y_list = [-0.9, 0.0, 1.3, 1.5, 0.9, 0.0, -1.5]
    

    index = np.random.randint(0,len(goal_x_list))

    '''#Stage 2
    goal_x_list = [0, 1.4, -1.5, 0, -1.5, 1.6]
    goal_y_list = [ -1.5, -1.5,  -1.5, 1.4, 1.4, 1.4]'''

    x = goal_x_list[index]
    y = goal_y_list[index]

    return x, y

def setTurtleBotPos(setPosPub, x, y, theta):
    initModel = ModelState()

    initModel.model_name = 'turtlebot3_burger'

    initModel.pose.position.x = x
    initModel.pose.position.y = y
    initModel.pose.position.z = 0.0

    [x_q,y_q,z_q,w_q] = quaternion_from_euler(0.0,0.0,radians(theta))

    initModel.pose.orientation.x = x_q
    initModel.pose.orientation.y = y_q
    initModel.pose.orientation.z = z_q
    initModel.pose.orientation.w = w_q

    setPosPub.publish(initModel)
    return ( x , y, theta )

'''
    getTurtleBotRotation 
    ROS function to calculate the the position of the turtlebot by using Odometry data
    
    return: x and y of the turtlebot as array
'''
def getPosition(odomMsg):
    x = odomMsg.pose.pose.position.x
    y = odomMsg.pose.pose.position.y
    arr = [x, y]
    return arr

###
def doTurtleBotAction(cmdVelPub, cmd_vel_linear, cmd_vel_angular):
    status = 'doTurtleBotAction => OK'

    #max_angular_vel = 1.5
    
    #ang_vel = ((5 - 1)/2 - action) * max_angular_vel * 0.5
    vel_cmd = Twist()
    vel_cmd.linear.x = cmd_vel_linear
    vel_cmd.angular.z = cmd_vel_angular
    cmdVelPub.publish(vel_cmd)
    return status
###
def turtleBotGoForward(cmdVelPub):
    velMsg = createCmdVelMsg(CONST_LINEAR_SPEED_FORWARD,CONST_ANGULAR_SPEED_FORWARD)
    cmdVelPub.publish(velMsg)
###
def turtleBotTurnRight(cmdVelPub):
    velMsg = createCmdVelMsg(CONST_LINEAR_SPEED_TURN,-CONST_ANGULAR_SPEED_TURN)
    cmdVelPub.publish(velMsg)
###
def turtleBotTurnLeft(cmdVelPub):
    velMsg = createCmdVelMsg(CONST_LINEAR_SPEED_TURN,+CONST_ANGULAR_SPEED_TURN)
    cmdVelPub.publish(velMsg)

def checkCrash(lidarDist):
    if lidarDist is not None:
        lidar_front_view = np.concatenate((lidarDist[(ANGLE_MIN + HORIZON_WIDTH):(ANGLE_MIN):-1],\
            lidarDist[(ANGLE_MAX): (ANGLE_MAX - HORIZON_WIDTH):-1]))
        param = np.linspace(1.2, 1, len(lidar_front_view) // 2)
        param = np.append(param, np.linspace(1, 1.2, len(lidar_front_view) // 2))
        if np.min( param * lidar_front_view ) < COLLISION_DISTANCE:
            return True
    return False

def  getLidarDist(msgScan):
    distances = np.array([])

    for i in range(len(msgScan.ranges)):
        if ( msgScan.ranges[i] > MAX_LIDAR_DISTANCE ):
            distance = MAX_LIDAR_DISTANCE
        elif ( msgScan.ranges[i] < msgScan.range_min ):
            distance = msgScan.range_min
        else:
            distance = msgScan.ranges[i]

        distances = np.append(distances, distance)
    # distances in [m]
    return distances

    '''
    Calculate euler distance of given two points
    return distance in float
    '''
def calcDistance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)