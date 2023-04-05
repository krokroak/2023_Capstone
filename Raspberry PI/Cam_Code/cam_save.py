import cv2 as cv
import numpy as np

cap = cv.VideoCapture('acsend.avi')

width = cap.get(cv.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv.CAP_PROP_FRAME_HEIGHT)
print("size : {0} x {1}".format(width, height))


fourcc = cv.VideoWriter_fourcc(*'XVID')
writer = cv.VideoWriter('acsenddge2.avi', fourcc, 15.0, (int(width), int(height)))

while cap.isOpened():
    success, frame = cap.read()
    if success:
        edge= cv.Canny(frame, 70, 120)
        writer.write(edge)
        cv.imshow('Video', edge)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break
    
cap.release()
writer.release()
cv.destroyAllWindows()