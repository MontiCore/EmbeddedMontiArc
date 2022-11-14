#!/usr/bin/env python

import rospy
import math
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
    velMsg.linear.y = 0  # you can delte this line see rl navigation (mantis) github
    velMsg.linear.z = 0  # you can delte this line see rl navigation (mantis) github
    velMsg.angular.x = 0 # you can delte this line see rl navigation (mantis) github
    velMsg.angular.y = 0 # you can delte this line see rl navigation (mantis) github
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

    initModel.twist.linear.x = 0.0  #try to delte this line and see what will happend
    initModel.twist.linear.y = 0.0  #try to delte this line and see what will happend
    initModel.twist.linear.z = 0.0  #try to delte this line and see what will happend
    
    initModel.twist.angular.x = 0.0 #try to delte this line and see what will happend
    initModel.twist.angular.y = 0.0 #try to delte this line and see what will happend
    initModel.twist.angular.z = 0.0 #try to delte this line and see what will happend

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

    initModel.twist.linear.x = 0.0  #try to delte this line and see what will happend
    initModel.twist.linear.y = 0.0  #try to delte this line and see what will happend
    initModel.twist.linear.z = 0.0  #try to delte this line and see what will happend

    initModel.twist.angular.x = 0.0 #try to delte this line and see what will happend
    initModel.twist.angular.y = 0.0 #try to delte this line and see what will happend
    initModel.twist.angular.z = 0.0 #try to delte this line and see what will happend

    setPosPub.publish(initModel)
    return ( x , y, theta )

'''
    getTurtleBotRotation 
    ROS function to calculate the orientation of the turtlebot by using Odometry data
    
    return: the orientation of the turtlebot as yaw
'''
def getTurtleBotRotation(odomMsg):
    orientation_q = odomMsg.pose.pose.orientation
    orientation_list = [ orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    return yaw

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

'''
    getHeading
    Calculate heading angle from turtlebot to goal
    
    return: angle in float
'''

def getHeading(turtle_x, turtle_y, goal_x, goal_y, yaw):
    
    targetAngle = math.atan2(goal_y - turtle_y, goal_x - turtle_x)
    heading = targetAngle - yaw
    if heading > math.pi:
        heading -= 2 * math.pi
    elif heading < -math.pi:
        heading += 2 * math.pi
    return round(heading, 2)

'''
    getReward
    Calculate the reward of an action.
    
    return: reward and terminate state
'''
def getReward(action, heading, current_distance, goal_distance, obstacle_min_range, crash):
    
    reward = 0
    terminal_state = False
    angle = -math.pi / 4 + heading + (math.pi / 8 * action) + math.pi / 2
    yaw_reward = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
    
    try:
        distance_rate = 2 ** (current_distance / goal_distance)
    except Exception:
        print("Overflow err CurrentDistance = ", current_distance, " TargetDistance = ", goal_distance)
        distance_rate = 2 ** (current_distance // goal_distance)
    
    reward += ((round(yaw_reward * 3, 2)) * distance_rate) #multiply with the number of action, here: 3 
    
    '''if obstacle_min_range < 0.5: 
        reward += -5
    else:
        reward += 0'''#for later training 
        
    if crash: # could be a crash and near to goal? maybe if else
        rospy.loginfo("Crash!!")
        terminal_state = True
        reward += -150
    
    if current_distance <= 0.2:
        rospy.loginfo("Goal!!")
        terminal_state = True
        reward += 200
    return reward, terminal_state

###
def doTurtleBotAction(cmdVelPub, action):
    status = 'doTurtleBotAction => OK'
    if action == 2:
        turtleBotGoForward(cmdVelPub)
    elif action == 1:
        turtleBotTurnLeft(cmdVelPub)
    elif action == 0:
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

def checkDiff(a, b, tolerant_value):
    if abs(a-b) < tolerant_value:
        return True
    else:
        return False


    '''
    Calculate euler distance of given two points
    return distance in float
    '''
def calcDistance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)