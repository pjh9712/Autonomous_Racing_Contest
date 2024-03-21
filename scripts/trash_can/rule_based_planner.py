#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rospy
from math import cos, sin, pi, sqrt, pow, atan2
import numpy as np
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, ObjectStatusList, EventInfo, Lamps
from morai_msgs.srv import MoraiEventCmdSrv

from pid_controller import pidControl
from pure_pursuit import pure_pursuit
from longitudinal_control_vel import velocityPlanning
from object_detector import object_detector

class rule_based_planner:
    def __init__(self):
        rospy.init_node('rule_based_planner', anonymous=True)

        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        rospy.Subscriber("/local_path", Path, pure_pursuit.path_callback)
        rospy.Subscriber("/odom", Odometry, pure_pursuit.odom_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, pure_pursuit.status_callback)

        rospy.Subscriber("/Object_topic", ObjectStatusList, self.object_callback)
        
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd', CtrlCmd, queue_size=1)

        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

        self.is_path = False
        self.is_odom = False
        self.is_status = False
        
        self.is_global_path = False
        
        self.target_velocity = 40 # morive max_speed": 60, default : 40
        # self.is_go = False
        # self.time = 1
        # self.parking_time=1

        self.pid = pidControl()
        self.vel_planning = velocityPlanning(self.target_velocity / 3.6, 0.15)
        self.pure_pursuit = pure_pursuit()
        self.object_detector = object_detector()
        
        
        while True:
            if self.is_global_path == True:
                self.velocity_list = self.vel_planning.curvedBaseVelocity(self.global_path, 50)
                rospy.loginfo('Recieved')
                break
            else:
                rospy.loginfo('Waiting global path data')

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            
            ## can not enter to if
            if self.is_path == True and self.is_odom == True and self.is_status == True:
                prev_time = time.time()
                ## Added for debug
                # rospy.loginfo("entered if")
                
                self.current_waypoint = pure_pursuit.get_current_waypoint(self.status_msg,self.global_path)
                self.target_velocity = self.velocity_list[self.current_waypoint]*3.6
                
                steering = pure_pursuit.calc_pure_pursuit()
                
                if self.is_look_forward_point :
                    self.ctrl_cmd_msg.steering = steering
                else : 
                    rospy.loginfo("no found forward point")
                    self.ctrl_cmd_msg.steering = steering
                    # self.ctrl_cmd_msg.steering = 0.0
                
                output = self.pid.pid(self.target_velocity,self.status_msg.velocity.x*3.6)
                
                nearest_dis, heading_difference = self.object_detector.nearest_cost()

                # same heading degree
                if heading_difference > 1.0:
                    if nearest_dis < 25.0:
                        self.ctrl_cmd_msg.accel = 0.0
                        self.ctrl_cmd_msg.brake = 1.0
                        rospy.loginfo('#############brake##############')

                    else:
                        if output > 0.0:
                            self.ctrl_cmd_msg.accel = output
                            self.ctrl_cmd_msg.brake = 0.0
                            # morive brake tunning
                        elif -5.0 < output <= 0.0:
                            self.ctrl_cmd_msg.accel = 0.0
                            self.ctrl_cmd_msg.brake = 0.0

                        else:
                            self.ctrl_cmd_msg.accel = 0.0
                            self.ctrl_cmd_msg.brake = -output
                else:
                    #rospy.loginfo("output' %s", output)
                    if output > 0.0:
                        self.ctrl_cmd_msg.accel = output
                        self.ctrl_cmd_msg.brake = 0.0

                    # morive brake tunning
                    elif -5.0 < output <= 0.0:
                        self.ctrl_cmd_msg.accel = 0.0
                        self.ctrl_cmd_msg.brake = 0.0

                    else:
                        self.ctrl_cmd_msg.accel = 0.0
                        self.ctrl_cmd_msg.brake = -output
                
                
                # if output > -17.0: #defaul 0.0
                #     self.ctrl_cmd_msg.accel = output
                #     self.ctrl_cmd_msg.brake = 0.0
                # else:
                #     self.ctrl_cmd_msg.accel = 0.0
                #     self.ctrl_cmd_msg.brake = -output
                #     print (output)
                    
                #TODO: (8) 제어입력 메세지 Publish
                # print('steering : ', steering)
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            rate.sleep()


    def call_service(self, gear_value):
        rospy.wait_for_service('Service_MoraiEventCmd')
        try:
            service_client = rospy.ServiceProxy('Service_MoraiEventCmd', MoraiEventCmdSrv)
            request_data = EventInfo()

            request_data.option = 2
            request_data.ctrl_mode = 3
            request_data.gear = gear_value
            lamps_data = Lamps()
            lamps_data.turnSignal = 0 
            lamps_data.emergencySignal = 0 
            request_data.lamps = lamps_data
            request_data.set_pause = False

            response = service_client(request_data)
            return response
        
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return None


    def object_callback(self, msg):
        self.is_object = True
        self.object_msg = msg


    def global_path_callback(self, msg):
        self.global_path = msg
        self.is_global_path = True

if __name__ == '__main__':
    try:
        test_track = rule_based_planner()
    except rospy.ROSInterruptException:
        pass
