from _21_MPU_Thread_ch1 import Angle_MPU6050_1
from _21_MPU_Thread_ch0 import Angle_MPU6050_0
import time
mpu6050L = Angle_MPU6050_1(0x68)  
mpu6050R = Angle_MPU6050_1(0x69)
#mpu6050M = Angle_MPU6050_0(0x68)

mpu6050L.start_measure_thread()
mpu6050R.start_measure_thread()
#mpu6050M.start_measure_thread()
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


while True:
    start = time.time()
    # Kalman 필터를 이용한 각도
    kalman_rollL = mpu6050L.get_kalman_roll()
    #kalman_pitchL = mpu6050L.get_kalman_pitch()
    kalman_rollR = mpu6050R.get_kalman_roll()
    #kalman_pitchR = mpu6050R.get_kalman_pitch()
    #kalman_rollM = mpu6050M.get_kalman_roll()
    Lr= int(round(kalman_rollL,3)*100)
    #Lp = int(round(kalman_pitchL,3)*100)
    Rr= int(round(kalman_rollR/2,3)*100)
    #Rp = int(round(kalman_pitchR,3)*100)
    #Mr= int(round(kalman_rollM,3)*100)
    #Mp = int(round(kalman_pitchR,3)*100)
    currentL = str(Lr)+ ' '
    currentR = str(Rr)+ ' '
    #currentM = str(Mr)+ ' '
    sendL = currentL.encode(encoding='UTF-8',errors='ignore')
    sendR = currentR.encode(encoding='UTF-8',errors='ignore')
    #stmR.write(sendR)
    stmL.write(sendL)
    
    time.sleep(0.01)
    print(currentL,' ',currentR) 
    end = time.time()
    #print(currentL, ' ', currentR, ' ',currentM )
    
    # 100Hz 제어 중 (UART는 40KHz까지 가능)
    # time 체크시 0.0006s 미만으로 나옴 -> 1666Hz (time.sleep 0.025, Uart Send)  Solo
    # time 체크 8.9884 e-05 s미만  = 0.000089884 ->  약 11125Hz  (No time Sleep, Uart Send, Dual Sensor)
    # time 체크 0.00025xxxxx s미만  =>  약 4000Hz  (0.001 time Sleep, Uart Send, Dual Sensor)


# MAX485는 라즈배리파이4 5V  + GND

# USB 연결 필수 (3.3V 넣어야함)

# 