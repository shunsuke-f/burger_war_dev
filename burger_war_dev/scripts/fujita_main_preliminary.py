#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs

import math

# Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
#import cv2


class SampleBot():

    waypoint_list = []
    # スタート地点付近
    waypoint_list.append([-0.9,0.0,290])
    waypoint_list.append([-0.9,0.0,0])

    # パティ横の壁
    waypoint_list.append([-0.9,0.5,45])
    waypoint_list.append([-0.72,0.72,45])
    waypoint_list.append([-0.5,0.9,45])

    waypoint_list.append([0,0.9,0])
    waypoint_list.append([0,0.9,225])
    waypoint_list.append([0,0.9,315])

    # カレー横の壁
    waypoint_list.append([0.5,0.9,315])
    waypoint_list.append([0.72,0.72,315])
    waypoint_list.append([0.9,0.5,315])

    waypoint_list.append([0.9,0,270])
    waypoint_list.append([0.9,0,135])
    waypoint_list.append([0.9,0,225])

    # チーズ横の壁
    waypoint_list.append([0.9,-0.5,225])
    waypoint_list.append([0.72,-0.72,225])
    waypoint_list.append([0.5,-0.9,225])

    waypoint_list.append([0,-0.9,180])
    waypoint_list.append([0,-0.9,45])
    waypoint_list.append([0,-0.9,135])

    # トマト横の壁
    waypoint_list.append([-0.5,-0.9,135])
    waypoint_list.append([-0.72,-0.72,135])
    waypoint_list.append([-0.9,-0.5,135])

    waypoint_list.append([-1.2,0,90])
    waypoint_list.append([-1.2,0,45])

    def __init__(self):
        
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        # subscriber
        self.pose_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.poseCallback)
#        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)


    def poseCallback(self, data):
        '''
        pose topic from amcl localizer
        update robot twist
        '''
        pose_x = data.pose.pose.position.x
        pose_y = data.pose.pose.position.y
        quaternion = data.pose.pose.orientation
        rpy = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        th = rpy[2]

        print("pose_x: {}, pose_y: {}, theta: {}".format(pose_x, pose_y, th))
        """
        th_xy = self.calcTargetTheta(pose_x,pose_y)
        
        th_diff = th_xy - th
        while not PI >= th_diff >= -PI:
            if th_diff > 0:
                th_diff -= 2*PI
            elif th_diff < 0:
                th_diff += 2*PI

        delta_th = self.calcDeltaTheta(th_diff)
        new_twist_ang_z = max(-0.3, min((th_diff + delta_th) * self.k , 0.3))
        
        self.twist.angular.z = new_twist_ang_z
        print("th: {}, th_xy: {}, delta_th: {}, new_twist_ang_z: {}".format(th, th_xy, delta_th, new_twist_ang_z))
        """


    def setGoal(self,x,y,yaw):#yaw[rad]
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()        

    def setGoal(self,goal_position):#yaw[degree]
        print(goal_position)
        x,y,yaw = goal_position[0], goal_position[1], math.radians(goal_position[2])
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()        


    def strategy(self):
        r = rospy.Rate(5) # change speed 5fps

        for item in self.waypoint_list:
            self.setGoal(item)

        for item in self.waypoint_list:
            self.setGoal(item)

        """
        self.setGoal(-0.5,0,0)
        self.setGoal(-0.5,0,math.radians(90))
        
        self.setGoal(0,0.5,0)
        self.setGoal(0,0.5,3.1415)
        
        self.setGoal(-0.5,0,-3.1415/2)
        
        self.setGoal(0,-0.5,0)
        self.setGoal(0,-0.5,3.1415)
#        """


if __name__ == '__main__':
    rospy.init_node('Samplerun')
    bot = SampleBot()
    bot.strategy()
