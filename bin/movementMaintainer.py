#!/usr/bin/env python

import numpy as np
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from math import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler

MAX_LIDAR_DISTANCE = 1.0 #limit the maximum lidar distance to 1 meter
COLLISION_DISTANCE = 0.14 # limit the lidar distance for collision
NEARBY_DISTANCE = 0.45

ZONE_0_LENGTH = 0.4
ZONE_1_LENGTH = 0.7

ANGLE_MAX = 360 - 1
ANGLE_MIN = 1 - 1
HORIZON_WIDTH = 75

# speed parameters
CONST_LINEAR_SPEED_FORWARD = 0.08
CONST_ANGULAR_SPEED_FORWARD = 0.0
CONST_LINEAR_SPEED_TURN = 0.06
CONST_ANGULAR_SPEED_TURN = 0.4


def stopTurtleBot(velPub):
    velMsg = createCmdVelMsg(0.0,0.0)
    velPub.publish(velMsg)
    
def createCmdVelMsg(v,w):
    velMsg = Twist()
    velMsg.linear.x = v
    velMsg.linear.y = 0
    velMsg.linear.z = 0
    velMsg.angular.x = 0
    velMsg.angular.y = 0
    velMsg.angular.z = w
    return velMsg
    
def setTurtleBotRandomPos(setPosPub):
    x_range = np.array([ 2.0, 2.0, -2.5, 1.0, -1.0, -0.4, 0.6, 0.6, -1.4, -1.4])
    y_range = np.array([-0.4, 0.6, -1.4, 0.6, 2.0, -1.4, 1.0, -1.0, 0.0, 2.0])
    theta_range = np.arange(0, 360, 15)

    index = np.random.randint(0,len(x_range))
    index_theta = np.random.randint(0,len(theta_range))

    x = x_range[index]
    y = y_range[index]
    theta = theta_range[index_theta]

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

    initModel.twist.linear.x = 0.0
    initModel.twist.linear.y = 0.0
    initModel.twist.linear.z = 0.0

    initModel.twist.angular.x = 0.0
    initModel.twist.angular.y = 0.0
    initModel.twist.angular.z = 0.0

    setPosPub.publish(initModel)
    return ( x , y, theta )

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

    initModel.twist.linear.x = 0.0
    initModel.twist.linear.y = 0.0
    initModel.twist.linear.z = 0.0

    initModel.twist.angular.x = 0.0
    initModel.twist.angular.y = 0.0
    initModel.twist.angular.z = 0.0

    setPosPub.publish(initModel)
    return ( x , y, theta )

def getTurtleBotPos(odomMsg):
    x = odomMsg.pose.pose.position.x
    y = odomMsg.pose.pose.position.y
    return ( x , y)

def getTurtleBotRotation(odomMsg):
    orientation_q = odomMsg.pose.pose.orientation
    orientation_list = [ orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    return yaw

###
def getReward(action, prev_action, lidar, prev_lidar, crash):
    if crash:
        terminal_state = True
        reward = -100
    else:
        lidar_front_view = np.concatenate((lidar[(ANGLE_MIN + HORIZON_WIDTH):(ANGLE_MIN):-1],lidar[(ANGLE_MAX):(ANGLE_MAX - HORIZON_WIDTH):-1]))
        prev_lidar_front_view = np.concatenate((prev_lidar[(ANGLE_MIN + HORIZON_WIDTH):(ANGLE_MIN):-1],prev_lidar[(ANGLE_MAX):(ANGLE_MAX - HORIZON_WIDTH):-1]))
        terminal_state = False
        # Reward from action taken = fowrad -> +0.2 , turn -> -0.1
        if action == 0:
            r_action = +0.2
        else:
            r_action = -0.1
        # Reward from crash distance to obstacle change
        W = np.linspace(0.9, 1.1, len(lidar_front_view) // 2)
        W = np.append(W, np.linspace(1.1, 0.9, len(lidar_front_view) // 2))
        if np.sum( W * ( lidar_front_view - prev_lidar_front_view) ) >= 0:
            r_obstacle = +0.2
        else:
            r_obstacle = -0.2
        # Reward from turn left/right change
        if ( prev_action == 1 and action == 2 ) or ( prev_action == 2 and action == 1 ):
            r_change = -0.8
        else:
            r_change = 0.0

        # Cumulative reward
        reward = r_action + r_obstacle + r_change

    return ( reward, terminal_state )

###
def doTurtleBotAction(cmdVelPub, action):
    status = 'doTurtleBotAction => OK'
    if action == 0:
        turtleBotGoForward(cmdVelPub)
    elif action == 1:
        turtleBotTurnLeft(cmdVelPub)
    elif action == 2:
        turtleBotTurnRight(cmdVelPub)
    else:
        status = 'doTurtleBotAction => INVALID ACTION'
        turtleBotGoForward(cmdVelPub)

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
###   
def turtleBotStop(cmdVelPub):
    velMsg = createCmdVelMsg(0.0,0.0)
    cmdVelPub.publish(velMsg)

def checkCrash(lidarDist):
    lidar_front_view = np.concatenate((lidarDist[(ANGLE_MIN + HORIZON_WIDTH):(ANGLE_MIN):-1],\
        lidarDist[(ANGLE_MAX): (ANGLE_MAX - HORIZON_WIDTH):-1]))
    param = np.linspace(1.2, 1, len(lidar_front_view) // 2)
    param = np.append(param, np.linspace(1, 1.2, len(lidar_front_view) // 2))
    if np.min( param * lidar_front_view ) < COLLISION_DISTANCE:
        return True
    else:
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

def checkDiff(a, b, tolerant_value):
    if abs(a-b) < tolerant_value:
        return True
    else:
        return False
    
def getPosition(odomMsg):
    x = odomMsg.pose.pose.position.x
    y = odomMsg.pose.pose.position.y
    arr = [x, y]
    return arr

# Check - goal near
def checkGoalNear(x, y, x_goal, y_goal):
    ro = sqrt( pow( ( x_goal - x ) , 2 ) + pow( ( y_goal - y ) , 2) )
    if ro < 0.3:
        return True
    else:
        return False