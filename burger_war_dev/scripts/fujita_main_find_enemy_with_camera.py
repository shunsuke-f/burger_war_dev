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

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import sys


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

        # image publisher 追加した
        self.image_pub = rospy.Publisher('processed_image', Image, queue_size=10)

        # camera subscribver
        cols = 640
        rows = 480
        self.img = np.full((rows, cols, 3), 0, dtype=np.uint8)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)

        self.is_enemy_in_image = False
        self.cx = 0
        self.cy = 0

    # camera image call back sample
    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)


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


    def find_enemy_with_camera(self):
        # AllSensorBotクラス の imageCallback関数で取得された画像データを取得
        bgr_image = self.img

        # HSV色空間に変換
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

        # 画像の二値化のための範囲指定。HSVで。
        lower = np.array([-30, 100, 50]) # red
        upper = np.array([30, 255, 255]) # red

        # 値が指定した範囲内の画素は255、範囲外の画素を0にする二値化
        mask_image = cv2.inRange(hsv_image, lower, upper)

        # 先程二値化した画像をマスク画像としてBGR画像を切り抜き
        processed_image = cv2.bitwise_and(bgr_image, bgr_image, mask=mask_image)

        # 重心を求める
        mom = cv2.moments(mask_image)
        cx, cy = 0, 0
        if "m00" in mom and "m10" in mom and "m01" in mom and mom["m00"] <> 0:
            cx = int(mom["m10"]/mom["m00"])
            cy = int(mom["m01"]/mom["m00"])
        print(cx, cy)

        # 求めた重心の位置を示すために紫色の点を描画
        color = (255, 0, 255)
        processed_image = cv2.circle(processed_image, (cx, cy), 3, color, -1)

        # 加工した画像をROS Topicの形式に変換してpublish
        image_msg = bot.bridge.cv2_to_imgmsg(processed_image, "bgr8")
        self.image_pub.publish(image_msg)

        is_enemy_in_image = False
        if cx > 0 and cy > 0:
            is_enemy_in_image = True

        return is_enemy_in_image, cx, cy



    def strategy(self):
        r = rospy.Rate(5) # change speed 5fps

        print("self.find_enemy_with_camera()")
        self.find_enemy_with_camera()

        """
        for item in self.waypoint_list:
            self.setGoal(item)

        for item in self.waypoint_list:
            self.setGoal(item)
#        """
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
