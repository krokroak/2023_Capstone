import cv2
import time
import numpy as np

def canny_webcam():
    #"Live capture frames from webcam and show the canny edge image of the captured frames."

    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()  # ret gets a boolean value. True if reading is successful (I think). frame is an
        # uint8 numpy.ndarray
        ori = frame     # 원본 이미지
        frame = cv2.GaussianBlur(frame, (7, 7), 2.5)  # 가우시안 블러
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # GrayScale로 변환
        edge = cv2.Canny(frame, 50,70)
        
        mask = np.full_like(ori,255)
        height, width = edge.shape[:2]
        idx= (width//2, height//2)
        
        #normal = cv2.seamlessClone(ori,edge,mask,idx,cv2.NORMAL_CLONE)
        #mix = cv2.seamlessClone(ori,edge,mask,idx,cv2.MIXED_CLONE)
        for i in range (height):
            for j in range (width):
                if edge[i][j] == 255:
                    ori[i][j] = 255
        cv2.imshow('Merged',ori)
        time.sleep(0.01)
        
        if cv2.waitKey(20) == ord('q'):  # Introduce 20 milisecond delay. press q to exit.
            cv2.destroyAllWindows()
            break
        

canny_webcam()