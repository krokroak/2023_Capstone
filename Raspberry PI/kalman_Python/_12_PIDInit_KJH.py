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
        self.output = 0
        self.anti_windup =0
        self.saturation = 15000



    def set_angle(self, ref_angle, angle):
        self.ref_angle = ref_angle
        self.angle = angle
        self.error_angle = self.ref_angle - self.angle
        self.set_dt()
   
    def set_dt(self):
        self.dt = time.time()-self.timer
        self.timer = time.time()
    
    def get_p_value(self):
        self.p_value = self.kp * self.error_angle
        return self.p_value
    
    
    def get_i_value(self):
        self.i_sum = (self.error_angle-self.anti_windup) + self.i_sum #안티 와인드 업 적용
        self.i_value = self.ki *self.i_sum * self.dt
        return self.i_value
    
    
    def get_d_value(self):
        self.error_dif = (self.error_angle - self.error_old) /self.dt
        self.d_value = self.kd * self.error_dif
        return self.d_value
    
    
    def get_output(self):
        self.output = self.get_p_value() + self.get_i_value() + self.get_d_value()
        if(self.output> self.saturation):
            self.anti_windup = (self.output - self.saturation)/self.kp #안티 와인드업을 할 때 사용할 리미터를 걸었을때의 출력차이 값 계산
            self.output = self.saturation
        elif(self.output< -self.saturation) :
            self. anti_windup = (self.output + self.saturation)/self.kp #안티 와인드업을 할 때 사용할 리미터를 걸었을때의 출력차이 값 계산
            self.output = -self.saturation
        return self.output
