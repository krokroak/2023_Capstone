from _21_MPU_Thread_ch1 import Angle_MPU6050_1
from _21_MPU_Thread_ch0 import Angle_MPU6050_0
from _12_PIDInit import PID
import time
mpu6050L = Angle_MPU6050_1(0x68)  
#mpu6050R = Angle_MPU6050_1(0x69)
mpu6050M = Angle_MPU6050_0(0x68)
import threading


mpu6050L.start_measure_thread()
#mpu6050R.start_measure_thread()
mpu6050M.start_measure_thread()
# 끝!!! 이제 쓰레드가 돌면서 mpu6050L의 변수에 값을 계속 update한다.
import serial

#stmL = serial.Serial(port= '/dev/ttyACM0',baudrate=19200,timeout=1.0,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE)
#stmR = serial.Serial(port= '/dev/ttyACM1',baudrate=19200,timeout=1.0,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE)
time.sleep(0.1)
# Accel을 이용한 각도
#accel_roll = mpu6050L.get_accel_roll()
#accel_pitch = mpu6050L.get_accel_pitch()
# Gyro를 이용한 각도
#gyro_roll = mpu6050L.get_gyro_roll()
#gyro_pitch = mpu6050L.get_gyro_pitch()
#gyro_yaw = mpu6050L.get_gyro_yaw()
# DMP를 이용한 각도
#DMP_roll = mpu6050L.get_DMP_roll()
#DMP_pitch = mpu6050L.get_DMP_pitch()
#DMP_yaw = mpu6050L.get_DMP_yaw()
# 상보필터를 이용한 각도
#compl_roll = mpu6050L.get_complementary_roll()
#compl_pitch = mpu6050L.get_complementary_pitch()
def st_send(stm,angle):
    current = str(angle) + ' '
    tx = current.encode(encoding='UTF-8',errors='ignore')
    stm.write(tx)
    print(current)
    print(' ')

Left_PID = PID('LEFT',7,20,0.002)
Right_PID = PID('RIGHT',10,20,0.002)
    

while True:
    Left_PID.set_dt()
    #Right_PID.set_dt()

    # Kalman 필터를 이용한 각도
    kalman_rollL = mpu6050L.get_kalman_roll()
    #kalman_pitchL = mpu6050L.get_kalman_pitch()
    #kalman_rollR = mpu6050R.get_kalman_roll()
    #kalman_pitchR = mpu6050R.get_kalman_pitch()
    kalman_rollM = mpu6050M.get_kalman_roll()
    #kalman_pitchM = mpu6050R.get_kalman_pitch()


    Left_PID.set_angle(int(round(kalman_rollM,3)),int(round(kalman_rollL,3)))
    #Right_PID.set_angle(int(round(kalman_rollM,3)*100),int(round(kalman_rollR,3)*100))
    #Lp = int(round(kalman_pitchL,3)*100)
    #Rp = int(round(kalman_pitchR,3)*100)
    #Mr= int(round(kalman_rollM,3)*100)
    #Mp = int(round(kalman_pitchR,3)*100)
    
    #currentM = str(Mr)+ ' '
    #th1 = threading.Thread(target= st_send(stmL,Left_PID.get_input()))
    #th2 = threading.Thread(target= st_send(stmR,Right_PID.get_input()))
    
    #th1.start()
    #th2.start()
    #th1.join()
    #th2.join()
    time.sleep(0.01)
    print(kalman_rollL,kalman_rollM,str(Left_PID.get_input()))
    #print(str(send2-send1))
    