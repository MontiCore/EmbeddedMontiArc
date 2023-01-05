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

    vel_cmd = Twist()
    vel_cmd.linear.x = cmd_vel_linear
    vel_cmd.angular.z = cmd_vel_angular
    cmdVelPub.publish(vel_cmd)
    return status

    '''
    Calculate euler distance of given two points
    return distance in float
    '''
def calcDistance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)