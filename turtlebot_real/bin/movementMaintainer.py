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
    velMsg.linear.y = 0  # you can delte this line see rl navigation (mantis) github
    velMsg.linear.z = 0  # you can delte this line see rl navigation (mantis) github
    velMsg.angular.x = 0 # you can delte this line see rl navigation (mantis) github
    velMsg.angular.y = 0 # you can delte this line see rl navigation (mantis) github
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
    rospy.loginfo("heading" + str(heading))
    rospy.loginfo("action" + str(action))
    reward = 0
    terminal_state = False
    angle = -math.pi / 4 + heading + (math.pi / 8 * action) + math.pi / 2
    rospy.loginfo("angle" + str(angle))
    yaw_reward = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
    rospy.loginfo("yaw_reward" + str(yaw_reward))
    goal_reached = False
    
    try:
        distance_rate = 2 ** (current_distance / goal_distance)
        rospy.loginfo("distance_rate" + str(distance_rate))
    except Exception:
        print("Overflow err CurrentDistance = ", current_distance, " TargetDistance = ", goal_distance)
        distance_rate = 2 ** (current_distance // goal_distance)
    
    reward += ((round(yaw_reward * 5, 2)) * distance_rate) #multiply with the number of action, here: 3 
    rospy.loginfo("reward_distance_rate" + str(reward))
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
        goal_reached = True
        terminal_state = True
        reward += 200
    
    rospy.loginfo("reward" + str(reward))
    return reward, terminal_state, goal_reached

###
def doTurtleBotAction(cmdVelPub, action):
    status = 'doTurtleBotAction => OK'

    max_angular_vel = 1.5
    
    ang_vel = ((5 - 1)/2 - action) * max_angular_vel * 0.5
    vel_cmd = Twist()
    vel_cmd.linear.x = 0.08
    vel_cmd.angular.z = ang_vel
    cmdVelPub.publish(vel_cmd)

    return status

    '''
    Calculate euler distance of given two points
    return distance in float
    '''
def calcDistance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)