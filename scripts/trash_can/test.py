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
#-----
# import os
# import sys
# current_path = os.path.dirname(os.path.realpath(__file__))
# sys.path.append(current_path)

# from lib.mgeo.class_defs import *
# sys.path.insert(0, '~/ProgrammersDevcourse_Morai/morive_refactoring_ws/src/controller')

class test:
    def __init__(self):

        rospy.init_node('test', anonymous=True)
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd', CtrlCmd, queue_size=1)
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 2

        self.is_path = False
        self.is_odom = False
        self.is_status = False
        
        self.is_global_path = False
        self.is_go = False
        
        rate = rospy.Rate(30)  # 30hz


        self.target_velocity = 40 # morive max_speed": 60, default : 40

        while not rospy.is_shutdown():
            response = self.call_service(4)
        
            self.ctrl_cmd_msg.accel = 2.0
            self.ctrl_cmd_msg.brake = 0.0
            self.ctrl_cmd_msg.steering = -1.0
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



if __name__ == '__main__':
    try:
        test1 = test()

    except rospy.ROSInterruptException:
        pass
