#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from math import sqrt

class velocityPlanning:
    def __init__ (self,car_max_speed, road_friciton):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friciton

    def curvedBaseVelocity(self, gloabl_path, point_num):
        out_vel_plan = []

        for i in range(0,point_num):
            out_vel_plan.append(self.car_max_speed * 0.45)
            # out_vel_plan.append(self.car_max_speed * 0.65)

        for i in range(point_num, len(gloabl_path.poses)- point_num):
            x_list = []
            y_list = []
            for box in range(-point_num, point_num):
                x = gloabl_path.poses[i+box].pose.position.x
                y = gloabl_path.poses[i+box].pose.position.y
                x_list.append([-2*x, -2*y ,1])
                y_list.append((-x*x) - (y*y))

            #TODO: (6) 도로의 곡률 계산
            x_matrix = np.array(x_list)
            y_matrix = np.array(y_list)
            x_trans = x_matrix.T

            a_matrix = np.linalg.pinv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
            # a_matrix = np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix) # error
            a = a_matrix[0]
            b = a_matrix[1]
            c = a_matrix[2]
            r = sqrt(a*a+b*b-c)

            #TODO: (7) 곡률 기반 속도 계획
            v_max = sqrt(r*9.8*self.road_friction)
            
            # 감속 계수 (곡률이 더 클 때 더 많이 감속)
            deceleration_factor = 0.7  # 감속 조절 0.65 -> 0.8 -> 0.65
            v_max *= deceleration_factor
        
            if v_max > self.car_max_speed:
                v_max = self.car_max_speed
                
            out_vel_plan.append(v_max)

        # for i in range(len(gloabl_path.poses) - point_num, len(gloabl_path.poses)-10):
        #     out_vel_plan.append(30) # 30 -> 20 

        # for i in range(len(gloabl_path.poses) - 10, len(gloabl_path.poses)):
        #     out_vel_plan.append(0)

        return out_vel_plan
