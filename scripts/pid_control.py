#!/usr/bin/env python3
# -*- coding: utf-8 -*-

class pidControl:
    def __init__(self):
        self.p_gain = 1.45 # default 0.3 0.5 -> 1.0
        self.i_gain = 0.1 # 0.01 -> 0.03
        self.d_gain = 1.2 # default 0.03 0.9 -> 0.7 -> 1.0
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.02 # 

    def pid(self,target_vel, current_vel):
        error = target_vel - current_vel

        #TODO: (5) PID 제어 생성
        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error-self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output
