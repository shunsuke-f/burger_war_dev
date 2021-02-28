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


# respect is_point_enemy freom team rabbit
# https://github.com/TeamRabbit/burger_war
class EnemyDetector:
    '''
    Lidarのセンサ値から簡易的に敵を探す。
    obstacle detector などを使ったほうがROSらしいがそれは参加者に任せます。
    いろいろ見た感じ Team Rabit の実装は綺麗でした。
    方針
    実測のLidarセンサ値 と マップと自己位置から期待されるLidarセンサ値 を比較
    ズレている部分が敵と判断する。
    この判断は自己位置が更新された次のライダーのコールバックで行う。（フラグで管理）
    0.7m 以上遠いところは無視する。少々のズレは許容する。
    '''
    def __init__(self):
        self.max_distance = 0.7
        self.thresh_corner = 0.25
        self.thresh_center = 0.35

        self.pose_x = 0
        self.pose_y = 0
        self.th = 0
    
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

        is_near_enemy = sum(enemy_scan) > 5  # if less than 5 points, maybe noise
        if is_near_enemy:
            idx_l = [i for i, x in enumerate(enemy_scan) if x == 1]
            idx = idx_l[len(idx_l)/2]
            enemy_direction = idx / 360.0 * 2*math.pi
            enemy_dist = near_scan[idx]
        else:
            enemy_direction = None
            enemy_dist = None

        print("Enemy: {}, Direction: {}".format(is_near_enemy, enemy_direction))
        print("enemy points {}".format(sum(enemy_scan)))
        return is_near_enemy, enemy_direction, enemy_dist
        

    def is_point_enemy(self, dist, ang_deg):
        if dist == 0:
            return False

        ang_rad = ang_deg /360. * 2 * math.pi
        point_x = self.pose_x + dist * math.cos(self.th + ang_rad)
        point_y = self.pose_y + dist * math.sin(self.th + ang_rad)

        #フィールド内かチェック
        if   point_y > (-point_x + 1.53):
            return False
        elif point_y < (-point_x - 1.53):
            return False
        elif point_y > ( point_x + 1.53):
            return False
        elif point_y < ( point_x - 1.53):
            return False

        #フィールド内の物体でないかチェック
        len_p1 = math.sqrt(pow((point_x - 0.53), 2) + pow((point_y - 0.53), 2))
        len_p2 = math.sqrt(pow((point_x - 0.53), 2) + pow((point_y + 0.53), 2))
        len_p3 = math.sqrt(pow((point_x + 0.53), 2) + pow((point_y - 0.53), 2))
        len_p4 = math.sqrt(pow((point_x + 0.53), 2) + pow((point_y + 0.53), 2))
        len_p5 = math.sqrt(pow(point_x         , 2) + pow(point_y         , 2))

        if len_p1 < self.thresh_corner or len_p2 < self.thresh_corner or len_p3 < self.thresh_corner or len_p4 < self.thresh_corner or len_p5 < self.thresh_center:
            return False
        else:
            #print(point_x, point_y, self.pose_x, self.pose_y, self.th, dist, ang_deg, ang_rad)
            #print(len_p1, len_p2, len_p3, len_p4, len_p5)
            return True

# End Respect




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
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)

        # 位置の初期値
        self.pose_x = 0
        self.pose_y = 0
        self.pose = 0

        # 敵検知
#        self.pose_twist = Twist()
#        self.pose_twist.linear.x = self.speed; self.pose_twist.linear.y = 0.; self.pose_twist.linear.z = 0.
#        self.pose_twist.angular.x = 0.; self.pose_twist.angular.y = 0.; self.pose_twist.angular.z = 0.

        self.is_near_enemy = False
        self.enemy_direction = None
        self.enemy_dist = None
#        self.near_enemy_twist = Twist()
#        self.near_enemy_twist.linear.x = self.speed; self.near_enemy_twist.linear.y = 0.; self.near_enemy_twist.linear.z = 0.
#        self.near_enemy_twist.angular.x = 0.; self.near_enemy_twist.angular.y = 0.; self.near_enemy_twist.angular.z = 0.

        self.is_initialized_pose = False
        self.enemy_detector = EnemyDetector()


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

        print("pose_x: {}, pose_y: {}, theta: {}".format(self.pose_x, self.pose_y, self.th))
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


    def lidarCallback(self, data):
        '''
        lidar scan use for bumper , and find enemy
        controll speed.x
        '''
        scan = data.ranges
        self.scan = scan
#        self.is_near_wall = self.isNearWall(scan)
        
        # enemy detection
        self.is_near_enemy, self.enemy_direction, self.enemy_dist = self.enemy_detector.findEnemy(scan, self.pose_x, self.pose_y, self.th)
#        if self.is_initialized_pose:
#            self.is_near_enemy, self.enemy_direction, self.enemy_dist = self.enemy_detector.findEnemy(scan, self.pose_x, self.pose_y, self.th)
        
#        if self.is_near_enemy:
#            self.updateNearEnemyTwist()


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
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            for item in self.waypoint_list:
                if self.is_near_enemy:
                    while self.is_near_enemy:
                        print("敵発見")
                        if not self.is_near_enemy:
                            break

#                    twist.linear.x = self.near_enemy_twist.linear.x
#                    twist.angular.z = self.near_enemy_twist.angular.z
                else:
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
