#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
This is ALL SENSOR use node.
Mainly echo sensor value in tarminal.
Please Use for your script base.

by Takuya Yamaguchi @dashimaki360
'''

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2

import numpy as np

class AllSensorBot(object):
    def __init__(self, 
                 use_lidar=False, use_camera=False, use_imu=False,
                 use_odom=False, use_joint_states=False):

        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)

        # image publisher 追加した
        self.image_pub = rospy.Publisher('processed_image', Image, queue_size=10)


        # lidar scan subscriber
        if use_lidar:
            self.scan = LaserScan()
            self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)

        # camera subscribver
        # please uncoment out if you use camera
        if use_camera:
            # for convert image topic to opencv obj
#            self.img = None # うまくいかなかった
            cols = 640
            rows = 480
            self.img = np.full((rows, cols, 3), 0, dtype=np.uint8)
            self.bridge = CvBridge()
            self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)

        # imu subscriber
        if use_imu:
            self.imu_sub = rospy.Subscriber('imu', Imu, self.imuCallback)

        # odom subscriber
        if use_odom:
            self.odom_sub = rospy.Subscriber('odom', Odometry, self.odomCallback)

        # joint_states subscriber
        if use_joint_states:
            self.odom_sub = rospy.Subscriber('joint_states', JointState, self.jointstateCallback)

    def strategy(self):
        '''
        calc Twist and publish cmd_vel topic
        '''
        r = rospy.Rate(1)

        while not rospy.is_shutdown():
            # update twist
            twist = Twist()
            twist.linear.x = 0.1; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0

            # publish twist topic
            self.vel_pub.publish(twist)
            r.sleep()


    # lidar scan topic call back sample
    # update lidar scan state
    def lidarCallback(self, data):
        self.scan = data
        rospy.loginfo(self.scan)

    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        """ # dockerの人はコメントアウト
        cv2.imshow("Image window", self.img)
        cv2.waitKey(1)
        """

    # imu call back sample
    # update imu state
    def imuCallback(self, data):
        self.imu = data
        rospy.loginfo(self.imu)

    # odom call back sample
    # update odometry state
    def odomCallback(self, data):
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y
        rospy.loginfo("odom pose_x: {}".format(self.pose_x))
        rospy.loginfo("odom pose_y: {}".format(self.pose_y))

    # jointstate call back sample
    # update joint state
    def jointstateCallback(self, data):
        self.wheel_rot_r = data.position[0]
        self.wheel_rot_l = data.position[1]
        rospy.loginfo("joint_state R: {}".format(self.wheel_rot_r))
        rospy.loginfo("joint_state L: {}".format(self.wheel_rot_l))

if __name__ == '__main__':
    rospy.init_node('all_sensor_sample')
    bot = AllSensorBot(use_lidar=False, use_camera=True, use_imu=False,
                       use_odom=False, use_joint_states=False)
#    bot.strategy()


    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        bgr_image = bot.img
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        lower = np.array([-30, 100, 50])
        upper = np.array([30, 255, 255])
        mask_image = cv2.inRange(hsv_image, lower, upper)
        processed_image = cv2.bitwise_and(bgr_image, bgr_image, mask=mask_image)

        edges = cv2.Canny(processed_image,100,200)
        cv2.imshow("edges image", edges)
        cv2.waitKey(1)
        sum_x, sum_y= 0, 0
        num = 0.00001 # 0除算を防ぐため
#        """
        for x in range(0, edges.shape[0]):
            for y in range(0, edges.shape[1]):
                if edges[x][y] > 0:
                    sum_x = sum_x + x
                    sum_y = sum_y + y
                    num = num + 1
        cx = int(sum_x/num)
        cy = int(sum_y/num)

        print(cx, cy)
#        """

        
        #重心求める
        """ # うまく行かなかった
        ret,thresh = cv2.threshold(processed_image,127,255,cv2.THRESH_BINARY)
        print(type(thresh))
        imgEdge,contours,hierarchy = cv2.findContours(thresh, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE)

        cnt = contours[0]
        M = cv2.moments(cnt)

        # 重心の座標
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        """

        color = (0, 255, 0)
        processed_image = cv2.circle(processed_image, (cy, cx), 3, color, -1)

        cv2.imshow("Image window", processed_image)
        cv2.waitKey(1)


        image_msg = bot.bridge.cv2_to_imgmsg(processed_image, "bgr8")
        bot.image_pub.publish(image_msg)

        r.sleep()
