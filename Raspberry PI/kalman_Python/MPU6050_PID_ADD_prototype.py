from _21_MPU_Thread import Angle_MPU6050
import time
mpu6050L = Angle_MPU6050(0x68) 
mpu6050R = Angle_MPU6050(0x69)

mpu6050L.start_measure_thread()
mpu6050R.start_measure_thread()
# 끝!!! 이제 쓰레드가 돌면서 mpu6050L의 변수에 값을 계속 update한다.
import serial

stmL = serial.Serial(port= '/dev/ttyAMA1',baudrate=19200,timeout=1.0,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE)
stmR = serial.Serial(port= '/dev/ttyAMA0',baudrate=19200,timeout=1.0,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE)
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
L_Pitch_target = 250 
R_Pitch_target = 250
L_Old = 0; L_Iterm =0
R_Old = 0 ; R_Iterm = 0

# PID
dt = 0.00025
Ws = 1/dt
Kp = 1
Ki = 1
Kd = 0
while True:
    start = time.time()
    # Kalman 필터를 이용한 각도
    kalman_rollL = mpu6050L.get_kalman_roll()
    kalman_pitchL = mpu6050L.get_kalman_pitch()
    kalman_rollR = mpu6050R.get_kalman_roll()
    kalman_pitchR = mpu6050R.get_kalman_pitch()
    #Lr= int(round(kalman_rollL,3)*100)
    Lp = int(round(kalman_pitchL,3)*100)
    #Rr= int(round(kalman_rollR,3)*100)
    Rp = int(round(kalman_pitchR,3)*100)

    L_error = L_Pitch_target-Lp
    R_error = R_Pitch_target-Rp
   
    L_dif = Lp- L_Old
    R_dif = Rp- R_Old
    L_pterm = Kp*L_error
    R_Pterm = Kp*R_error
    L_Iterm = L_Iterm + Ki*L_error*dt
    R_Iterm = R_Iterm + Ki*R_error*dt
    L_Dterm = Kd*(L_dif/dt)
    R_Dterm = Kd*(R_dif/dt)
    
    L_result = L_pterm + L_Iterm + L_Dterm
    R_result = R_Pterm + R_Iterm + R_Dterm

    # P -> Error * Kp, D -> (Error/dt) *Kd , I -> errersum = errorsum +(Error)*dt  ->   errorsum   * Ki
    # PID는 Error를 0으로 수렴하는 것을 목표하는 제어
    #  
    current = str(L_result) + ' ' + str(R_result)
    #send = current.encode(encoding='UTF-8',errors='ignore')
    #stm.write(send)
    
    L_Old = Lp
    R_Old = Rp
    #time.sleep(0.001)
    end = time.time()
    #print(end-start, ' s')
    print(current)
    if L_error == 0:
        break
    # 100Hz 제어 중 (UART는 40KHz까지 가능)
    # time 체크시 0.0006s 미만으로 나옴 -> 1666Hz (time.sleep 0.025, Uart Send)  Solo
    # time 체크 8.9884 e-05 s미만  = 0.000089884 ->  약 11125Hz  (No time Sleep, Uart Send, Dual Sensor)
    # time 체크 0.00025xxxxx s미만  =>  약 4000Hz  (0.001 time Sleep, Uart Send, Dual Sensor)
