from smbus2 import SMBus
import numpy as np
import math
import time
from bitstring import Bits
bus = SMBus(1)
lDEV_ADDR = 0x68
rDEV_ADDR = 0x69

reg_gyro_xout_h = 0x43
reg_gyro_yout_h = 0x45
reg_gyro_zout_h = 0x47
reg_acc_xout_h = 0x3B
reg_acc_yout_h = 0x3D
reg_acc_zout_h = 0x3F
sen_gyro = 131.0
sen_acc = 16384
def Lread(reg):
    high = bus.read_byte_data(lDEV_ADDR,reg)
    low = bus.read_byte_data(lDEV_ADDR,reg+1)
    val = (high<<8) +low
    return val
def Rread(reg):
    high = bus.read_byte_data(rDEV_ADDR,reg)
    low = bus.read_byte_data(rDEV_ADDR,reg+1)
    val = (high<<8) +low
    return val
def twocom(val):
    s = Bits(uint=val,length=16)
    return s.int
def gyro_dps(val):
    return twocom(val)/sen_gyro
def acc_g(val):
    return twocom(val)/sen_acc

def dist(a,b):
    return math.sqrt((a*a)+(b*b))
def gen_x_rot(x,y,z):
    radians= math.atan2(x,dist(y,z))
    return radians
def gen_y_rot(x,y,z):
    radians= math.atan2(y,dist(x,z))
    return radians
def gen_z_rot(x,y,z):
    radians= math.atan2(z,dist(x,y))
    return radians
#칼만 필터 관련 변수 및 함수
P = np.array([[0,0],[0,0]])
K = np.array([[0],[0]])
Q_angle = 0.0 ; Q_gyro = 0.0 ; R_measure = 0.0 ; 
angle = 0.0 ; bias = 0.0 ; 
deg = 0 ; deg_y =0
dt = 0.0 ; val = 0.0

bus.write_byte_data(lDEV_ADDR,0x6B,0b00000000)
#bus.write_byte_data(rDEV_ADDR,0x6B,0b00000000)
def Kalman_init(angle, gyro, measure):
    Q_angle = angle
    Q_gyro = gyro
    R_measure = measure

    angle = 0 
    bias = 0 

    P[0][0] = 0
    P[0][1] = 0
    P[1][0] = 0
    P[1][1] = 0

def get_Kalman(acc,gyro,dt):
    global angle 
    global bias
    angle += dt * (gyro - bias) 

    #Project the error covariance ahead

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle)

    P[0][1] -= dt * P[1][1] 

    P[1][0] -= dt * P[1][1] 

    P[1][1] += Q_gyro * dt 



    #Compute the Kalman gain

    S = P[0][0] + R_measure 

    K[0] = P[0][0] / S 

    K[1] = P[1][0] / S 



    #Update estimate with measurement z

    y = acc - angle 

    angle += K[0] * y 

    bias += K[1] * y 



    #Update the error covariance
    P_temp = np.zeros(2)
    P_temp = P[0][0], P[0][1]

    P[0][0] -= K[0] * P_temp[0] 
    P[0][1] -= K[0] * P_temp[1] 
    P[1][0] -= K[1] * P_temp[0] 
    P[1][1] -= K[1] * P_temp[1] 

    return angle
try:
    while True:
        lx =Lread(reg_acc_xout_h)
        ly =Lread(reg_acc_yout_h)
        lz =Lread(reg_acc_zout_h)
        laX = gen_x_rot(acc_g(lx),acc_g(ly),acc_g(lz))
        laY = gen_x_rot(acc_g(ly),acc_g(lx),acc_g(lz))
        Kalman_init(0.001, 0.003, 0.03)
        lxval = get_Kalman(laX, laX/131, 0.02)
        lyval = get_Kalman(laY,laY/131,0.02)
        data =str(lxval) + ' , ' + str(lyval)+  '$'

        print(data)
        time.sleep(0.02)
except KeyboardInterrupt():
    print("\n Interrupted")
except :
    print("\nClosing PORT")
finally:
    bus.close(0)