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

from std_msgs.msg import Bool, String, Int32
import numpy as np

from std_msgs.msg import Int32MultiArray

PI = math.pi



class EnemyPointsFinder():

    def __init__(self):
        
        self.max_distance = 0.8 # 0.7
#        self.thresh_corner = 0.6 # 0.25
#        self.thresh_center = 0.5 # 0.35

        self.pose_x = 0
        self.pose_y = 0
        self.th = 0

        # lidar scan
        self.scan = []
        self.is_near_wall = False

        # publisher
        self.is_enemy_points_pub = rospy.Publisher('is_enemy_points', Bool, queue_size=10)
        self.enemy_direction_pub = rospy.Publisher('enemy_direction', Int32, queue_size=10)


        # subscriber
        self.pose_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.poseCallback)
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)

        self.is_near_enemy = False
        self.enemy_direction = None
        self.enemy_dist = None

        self.is_initialized_pose = False


    def findEnemy(self, scan, pose_x, pose_y, th):
        '''
        input scan. list of lider range, robot locate(pose_x, pose_y, th)
        return is_near_enemy(BOOL), enemy_direction[rad](float)
        '''
        if not len(scan) == 360:
            return False
        
        # update pose
        self.pose_x = pose_x
        self.pose_y = pose_y
        self.th = th

        # drop too big and small value ex) 0.0 , 2.0 
        near_scan = [x if self.max_distance > x > 0.1 else 0.0 for x in scan]

        enemy_scan = [1 if self.is_point_enemy(x,i) else 0 for i,x in  enumerate(near_scan)]

        is_near_enemy = sum(enemy_scan) > 6  # if less than 5 points, maybe noise
        if is_near_enemy:
            idx_l = [i for i, x in enumerate(enemy_scan) if x == 1]
            idx = idx_l[len(idx_l)/2]
            enemy_direction_deg = idx
            enemy_direction = idx / 360.0 * 2*PI
            enemy_dist = near_scan[idx]
        else:
            enemy_direction_deg = None
            enemy_direction = None
            enemy_dist = None

#        print("Enemy: {}, Direction: {}, Direction[deg]: {}".format(is_near_enemy, enemy_direction, enemy_direction_deg))
#        print("enemy points {}".format(sum(enemy_scan)))

        self.is_enemy_points_pub.publish(is_near_enemy)
        self.enemy_direction_pub.publish(enemy_direction_deg)

        return is_near_enemy, enemy_direction, enemy_dist
        

    def is_point_enemy(self, dist, ang_deg):
        if dist == 0:
            return False

        ang_rad = ang_deg /360. * 2 * PI
        point_x = self.pose_x + dist * math.cos(self.th + ang_rad)
        point_y = self.pose_y + dist * math.sin(self.th + ang_rad)

        #フィールド内かチェック
        filed_size = 1.3 #1.53
        if   point_y > (-point_x + filed_size):
            return False
        elif point_y < (-point_x - filed_size):
            return False
        elif point_y > ( point_x + filed_size):
            return False
        elif point_y < ( point_x - filed_size):
            return False
        else:
            #フィールド内の物体でないかチェック
            #小さい方
            locate = 0.53 # 0.53
            radius = 0.3  #半径
            large_radius = 0.5  #半径
            if (pow((point_x - locate), 2) + pow((point_y - locate), 2)) < pow(radius, 2):
                return False
            elif (pow((point_x - locate), 2) + pow((point_y + locate), 2)) < pow(radius, 2):
                return False
            elif (pow((point_x + locate), 2) + pow((point_y - locate), 2)) < pow(radius, 2):
                return False
            elif (pow((point_x + locate), 2) + pow((point_y + locate), 2)) < pow(radius, 2):
                return False
            # 真ん中の大きい障害物
            elif (pow(point_x, 2) + pow(point_y, 2)) < pow(radius, 2):
                return False
            else:
                return True

        """
        locate = 0.60 # 0.53
        len_p1 = math.sqrt(pow((point_x - locate), 2) + pow((point_y - locate), 2))
        len_p2 = math.sqrt(pow((point_x - locate), 2) + pow((point_y + locate), 2))
        len_p3 = math.sqrt(pow((point_x + locate), 2) + pow((point_y - locate), 2))
        len_p4 = math.sqrt(pow((point_x + locate), 2) + pow((point_y + locate), 2))
        len_p5 = math.sqrt(pow(point_x         , 2) + pow(point_y         , 2))

        if len_p1 < self.thresh_corner or len_p2 < self.thresh_corner or len_p3 < self.thresh_corner or len_p4 < self.thresh_corner or len_p5 < self.thresh_center:
            return False
        else:
            #print(point_x, point_y, self.pose_x, self.pose_y, self.th, dist, ang_deg, ang_rad)
            #print(len_p1, len_p2, len_p3, len_p4, len_p5)
            return True
        """


    def poseCallback(self, data):
        '''
        pose topic from amcl localizer
        update robot twist
        '''
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y
        quaternion = data.pose.pose.orientation
        rpy = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))

        self.th = rpy[2]
        
        self.is_initialized_pose = True


    def lidarCallback(self, data):
        '''
        lidar scan use for bumper , and find enemy
        controll speed.x
        '''
        scan = data.ranges
        self.scan = scan
        self.is_near_wall = self.isNearWall(scan)
        
        # enemy detection
        if self.is_initialized_pose:
            self.is_near_enemy, self.enemy_direction, self.enemy_dist = self.findEnemy(scan, self.pose_x, self.pose_y, self.th)
        
#        if self.is_near_enemy:
#            self.updateNearEnemyTwist()


    def isNearWall(self, scan):
        if not len(scan) == 360:
            return False
        forword_scan = scan[:15] + scan[-15:]
        # drop too small value ex) 0.0
        forword_scan = [x for x in forword_scan if x > 0.1]
        if min(forword_scan) < 0.2:
            return True
        return False




if __name__ == '__main__':
    try:
        rospy.init_node('enemy_points_finder', anonymous=False)
        finder = EnemyPointsFinder()

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():

            rate.sleep()
    except rospy.ROSInterruptException:
        pass