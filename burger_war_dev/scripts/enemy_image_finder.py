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

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

from std_msgs.msg import Int32MultiArray




class EnemyImageFinder():


    def __init__(self):
        
        # image publisher 追加した
        self.image_pub = rospy.Publisher('processed_image', Image, queue_size=10)
        self.enemy_green_center_pub = rospy.Publisher('enemy_green_center', Int32MultiArray, queue_size=10)

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


    def execute(self):
        # AllSensorBotクラス の imageCallback関数で取得された画像データを取得
        bgr_image = self.img

        # HSV色空間に変換
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

        # 画像の二値化のための範囲指定。HSVで。
        lower = np.array([40, 100, 50]) # green
        upper = np.array([60, 255, 255]) # green

        # 値が指定した範囲内の画素は255、範囲外の画素を0にする二値化
        mask_image = cv2.inRange(hsv_image, lower, upper)

        # 先程二値化した画像をマスク画像としてBGR画像を切り抜き
        processed_image = cv2.bitwise_and(bgr_image, bgr_image, mask=mask_image)

        # 重心を求める
        mom = cv2.moments(mask_image)
        cx, cy = 0, 0
#        rospy.loginfo("面積 %f", mom["m00"])
        if mom["m00"] > 1000000:
            if "m00" in mom and "m10" in mom and "m01" in mom and mom["m00"] <> 0:
                cx = int(mom["m10"]/mom["m00"])
                cy = int(mom["m01"]/mom["m00"])
#        print(cx, cy)

        center = [cx, cy]
        array_for_for_publishing = Int32MultiArray(data=center)
        self.enemy_green_center_pub.publish(array_for_for_publishing)

        # 求めた重心の位置を示すために紫色の点を描画
        color = (255, 0, 255)
        processed_image = cv2.circle(processed_image, (cx, cy), 3, color, -1)

        # 加工した画像をROS Topicの形式に変換してpublish
        image_msg = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
        self.image_pub.publish(image_msg)





if __name__ == '__main__':
    try:
        rospy.init_node('enemy_image_finder', anonymous=False)
        finder = EnemyImageFinder()

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            finder.execute()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass