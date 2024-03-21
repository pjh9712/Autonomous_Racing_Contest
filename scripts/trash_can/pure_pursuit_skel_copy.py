#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys
import time
import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd,EgoVehicleStatus
import numpy as np
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler

from pid_control import pidControl
from longitudinal_control_vel import velocityPlanning

# advanced_purepursuit 은 차량의 차량의 종 횡 방향 제어 예제입니다.
# Purpusuit 알고리즘의 Look Ahead Distance 값을 속도에 비례하여 가변 값으로 만들어 횡 방향 주행 성능을 올립니다.
# 횡방향 제어 입력은 주행할 Local Path (지역경로) 와 차량의 상태 정보 Odometry 를 받아 차량을 제어 합니다.
# 종방향 제어 입력은 목표 속도를 지정 한뒤 목표 속도에 도달하기 위한 Throttle control 을 합니다.
# 종방향 제어 입력은 longlCmdType 1(Throttle control) 이용합니다.

# 노드 실행 순서 
# 1. subscriber, publisher 선언
# 2. 속도 비례 Look Ahead Distance 값 설정
# 3. 좌표 변환 행렬 생성
# 4. Steering 각도 계산
# 5. PID 제어 생성
# 6. 도로의 곡률 계산
# 7. 곡률 기반 속도 계획
# 8. 제어입력 메세지 Publish

class st :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        #TODO: (1) subscriber, publisher 선언h_callback)
        rospy.Subscriber("/global_path", Path, self.global_path_callback)  
        rospy.Subscriber("/local_path", Path, self.path_callback)
        # rospy.Subscriber("/lattice_path", Path, self.path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Ego_topic",EgoVehicleStatus, self.status_callback)
        
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd_0',CtrlCmd, queue_size=1)

        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

        ## edited for debug
        self.is_path = False
        self.is_odom = True
        self.is_status = False
        self.is_global_path = False
        
        self.forward_point = Point()
        self.current_postion = Point()

        self.vehicle_length = 4.470 # wheel base 2.7 Length(m) : 4.470

        self.K = 1.0
        
        self.target_velocity = 40
        
        ######### 
        self.pid = pidControl()
        self.vel_planning = velocityPlanning(self.target_velocity/3.6, 0.15)

        while True:
            if self.is_global_path == True:
                self.velocity_list = self.vel_planning.curvedBaseVelocity(self.global_path, 50) # ponit_num 50 -> 35
                # rospy.loginfo("Velocity List: %s", self.velocity_list[:25])
                rospy.loginfo('Recieved global path data')
                break
            else:
                pass
                # rospy.loginfo('Waiting global path data')

        rate = rospy.Rate(30) # 30 -> 50 -> 30 Hz
        
        while not rospy.is_shutdown():
            
            ## can not enter to if
            if self.is_path == True and self.is_odom == True and self.is_status == True:
                prev_time = time.time()
                ## Added for debug
                # rospy.loginfo("entered if")
                self.current_waypoint = self.get_current_waypoint(self.status_msg,self.global_path)
                # rospy.loginfo("current_waypoint: %s", self.current_waypoint)
                self.target_velocity = self.velocity_list[self.current_waypoint]*3.6
                # rospy.loginfo("target_velocity: %s", self.target_velocity)
                steering = self.calc_stanley_steering()
                # rospy.loginfo("steering: %s", steering)
                
                self.ctrl_cmd_msg.steering = steering
                
                output = self.pid.pid(self.target_velocity,self.status_msg.velocity.x*3.6)
                # rospy.loginfo("output: %s", output)
                
                # if output > -17.0: #defaul 0.0
                #     self.ctrl_cmd_msg.accel = output
                #     self.ctrl_cmd_msg.brake = 0.0
                # else:
                #     self.ctrl_cmd_msg.accel = 0.0
                #     self.ctrl_cmd_msg.brake = -output
                #     print (output)
                
                if output > 0.0:
                    self.ctrl_cmd_msg.accel = output # output  0 ~ 1.0
                    self.ctrl_cmd_msg.brake = 0.0
                    # morive brake tunning
                elif -5.0 < output <= 0.0:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = 0.0
                else:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = -output # output 0 ~ 1.0
                    
                ## Stop at end point
                if(self.status_msg.position.x > 159.0):
                    self.ctrl_cmd_msg.steering = 0.0
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = 1.0
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                    print("Finished.")
                    print("distance from barrels :", 171.34324645996094-self.status_msg.position.x-self.vehicle_length)
                
                #TODO: (8) 제어입력 메세지 Publish
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            rate.sleep()

    def path_callback(self,msg):
        self.is_path=True
        self.path=msg
        
    def status_callback(self,msg): ## Vehicl Status Subscriber 
        self.is_status=True
        self.status_msg=msg
        self.vehicle_yaw=msg.heading - 90
        self.current_postion.x=msg.position.x
        self.current_postion.y=msg.position.y
        # rospy.loginfo("status_msg_heading : %s", self.vehicle_yaw)  
    
    def odom_callback(self,msg):
        self.is_odom=True
        # self.odom_msg = msg
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y
        # rospy.loginfo("odom_vehicle_yaw: %s", self.vehicle_yaw)
    
    def global_path_callback(self,msg):
        self.global_path = msg
        self.is_global_path = True
        
    def get_current_waypoint(self,ego_status,global_path):
        min_dist = float('inf')        
        currnet_waypoint = -1
        for i,pose in enumerate(global_path.poses):
            dx = ego_status.position.x - pose.pose.position.x
            dy = ego_status.position.y - pose.pose.position.y

            dist = sqrt(pow(dx,2)+pow(dy,2))
            if min_dist > dist :
                min_dist = dist
                currnet_waypoint = i
        return currnet_waypoint

    def calc_stanley_steering(self):
        print("def")
        nearest_point, nearest_index = self.find_nearest_point(self.path)
        if nearest_index == -1:
            return 0.0

        cross_track_error = self.calc_cross_track_error(nearest_point)
        path_heading = self.calc_path_heading(nearest_index)
        heading_error = path_heading - np.deg2rad(self.vehicle_yaw)
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

        steering_angle = heading_error + np.arctan2(self.K * cross_track_error, self.status_msg.velocity.x)

        return steering_angle

    def find_nearest_point(self, path):
        min_distance = float('inf')
        nearest_point = None
        nearest_index = -1
        for i, pose in enumerate(path.poses):
            distance = self.calc_distance(self.current_position, pose.pose.position)
            if distance < min_distance:
                min_distance = distance
                nearest_point = pose.pose.position
                nearest_index = i
        return nearest_point, nearest_index

    def calc_cross_track_error(self, nearest_point):
        dx = nearest_point.x - self.current_position.x
        dy = nearest_point.y - self.current_position.y
        print(dx**2 + dy**2)
        return sqrt(dx**2 + dy**2)

    def calc_path_heading(self, nearest_index):
        current_point = self.path.poses[nearest_index].pose.position
        if nearest_index + 1 < len(self.path.poses):
            next_point = self.path.poses[nearest_index + 1].pose.position
        else:
            next_point = current_point
        return np.arctan2(next_point.y - current_point.y, next_point.x - current_point.x)

    def calc_distance(self, point1, point2):
        print(sqrt((point2.x - point1.x)**2 + (point2.y - point1.y)**2))
        return sqrt((point2.x - point1.x)**2 + (point2.y - point1.y)**2)

if __name__ == '__main__':
    try:
        test_track=st()
    except rospy.ROSInterruptException:
        pass
