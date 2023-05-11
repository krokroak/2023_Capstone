from _21_MPU_Thread import Angle_MPU6050
import time
import multiprocessing
import threading
mpu6050L = Angle_MPU6050(0x68) 
mpu6050R = Angle_MPU6050(0x69)

mpu6050L.start_measure_thread()
mpu6050R.start_measure_thread()
# 끝!!! 이제 쓰레드가 돌면서 mpu6050L의 변수에 값을 계속 update한다.
import serial

stmL = serial.Serial(port= '/dev/ttyACM0',baudrate=19200,timeout=1.0,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE)
stmR = serial.Serial(port= '/dev/ttyACM1',baudrate=19200,timeout=1.0,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE)
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
    current = str(angle/2) + ' '
    tx = current.encode(encoding='UTF-8',errors='ignore')
    stm.write(tx)
    print(current)
    print(' ')



while True:
    start = time.time()
    # Kalman 필터를 이용한 각도
    kalman_rollL = mpu6050L.get_kalman_roll()
    #kalman_pitchL = mpu6050L.get_kalman_pitch()
    kalman_rollR = mpu6050R.get_kalman_roll()
    #kalman_pitchR = mpu6050R.get_kalman_pitch()
    Lr= int(round(kalman_rollL,3)*100)
    #Lp = int(round(kalman_pitchL,3)*100)
    Rr= int(round(kalman_rollR,3)*100)
    #Rp = int(round(kalman_pitchR,3)*100)
    th1 = threading.Thread(target= st_send(stmL,Lr))
    th2 = threading.Thread(target= st_send(stmR,Rr))
    
    th1.start()
   
    th2.start()
    th1.join()
    th2.join()
    time.sleep(0.01)
    end = time.time()
    #print(str(send2-send1))
    