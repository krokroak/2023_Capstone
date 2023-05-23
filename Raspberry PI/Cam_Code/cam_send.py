import serial
import cv2 
import time
import numpy as np


#stm = serial.Serial(port = '/dev/ttyAMA1',baudrate = 19200,timeout=1.0,
#                  bytesize=serial.EIGHTBITS,
#                    parity=serial.PARITY_NONE,
#                    stopbits=serial.STOPBITS_ONE)
time.sleep(0.5)
#stm.reset_input_buffer()
def canny_serial():
    power = "Target Pixel Detected "        #전송용 메세지
    cam = cv2.VideoCapture(0)
    while True:
        
        ret, frame = cam.read()
        ori = frame     # 원본 이미지
        frame = cv2.GaussianBlur(frame, (7, 7), 1.5)  # 가우시안 블러
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # GrayScale로 변환
        edge = cv2.Canny(frame, 20,40)      #경계선 추출
        
        
        mask = np.full_like(ori,255)
        
        height, width = edge.shape[:2]
        # 컬러 이미지에 경계선 넣는 부분 (느려서 안씀)
        #for i in range (height):
        #    for j in range (width):
        #        if edge[i][j] == 255:
        #           ori[i][j] = 255
        w_c = width // 2    #가로의 중심
        h_c = height // 2   # 세로의 중심
        
        # 중심을 표시하기 위해서 주변에 도트를 표시
        edge[h_c][w_c]=255
        edge[h_c+10][w_c+10]=255
        edge[h_c-10][w_c-10]=255
        edge[h_c][w_c+10]=255
        edge[h_c-10][w_c]=255
        edge[h_c-10][w_c+10]=255
        edge[h_c+10][w_c-10]=255
        edge[h_c][w_c-10]=255
        edge[h_c+10][w_c]=255

        #경계선 이미지 보여주기
        cv2.imshow('Edged',edge)
        time.sleep(0.03)

        #중심 주변부에 경계선을 감지하면
        if edge[h_c+5][w_c]==255 | edge[h_c-5][w_c]==255 |edge[h_c][w_c+5]==255| edge[h_c][w_c-5]==255:
            print(power)
            #window 노트북으로 전송
            #stm.write(power.encode(encoding='UTF-8',errors='ignore'))
            time.sleep(0.1)
        if cv2.waitKey(10) == ord('q'):  # Introduce 10 milisecond delay. press q to exit.
            cv2.destroyAllWindows()
            break
canny_serial()
    