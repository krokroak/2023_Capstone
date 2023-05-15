import math
import time

class PID:
    def __init__(self,part,kp,ki,kd):
        self.part = part
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.timer = 0
        self.error_angle = 0
        self.ref_angle = 0
        self.angle = 0
        self.p_value = 0
        self.dt = 0
        self.i_sum = 0
        self.i_value =0
        self.error_old = 0
        self.error_dif = 0 
        self.d_value = 0
        self.input = 0


    def set_angle(self, ref_angle, angle):
        self.ref_angle = ref_angle
        self.angle = angle
        self.error_angle = self.ref_angle - self.angle
       
   
    def set_dt(self):
        self.dt = time.time()-self.timer
        self.timer = time.time()
        return self.dt
    
    def get_p_value(self):
        self.p_value = self.kp * self.error_angle
        return self.p_value
    
    
    def get_i_value(self):
        self.i_sum = (self.error_angle * self.dt) + self.i_sum
        self.i_value = self.ki *self.i_sum
        return self.i_value
    
    
    def get_d_value(self):
        self.error_dif = (self.error_angle - self.error_old) /self.dt
        self.d_value = self.kd* self.error_dif
        return self.d_value
    
    
    def get_input(self):
        self.input = self.get_p_value() + self.get_i_value() + self.get_d_value()
        self.error_old = self.error_angle
        return self.input
