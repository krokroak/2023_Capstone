import cv2 as cv
import numpy as np
import time
import sys
import math


def main(argv):
    cam = cv.VideoCapture('acsend.avi')
    while True:
        
        ret,source = cam.read()
        if source is None:
            print('Opening Error: Could not open')
            return -1
        
        demand = cv.Canny(source,50,120,None,3)
        
        copy_img = cv.cvtColor(demand,cv.COLOR_GRAY2BGR)
        copy_imgP= np.copy(copy_img)
        
        Normal_line = cv.HoughLines(demand,1,np.pi/180,60,None,0,0)
        if Normal_line is None:
            for i in range(0,len(Normal_line)):
                r = Normal_line[i][0][0]
                t = Normal_line[i][0][1]
                cos = math.cos(t)
                sin = math.sin(t)
                x0 = cos * r
                y0 = sin * r
                lt1 = (int(x0+1000*(-sin)), int(y0+1000*(cos)))
                lt2 = (int(x0-1000*(-sin)), int(y0-1000*(cos)))
                cv.line(copy_img,lt1,lt2,(0,0,255),3,cv.LINE_AA)
        cv.imshow("Source",source)
        cv.imshow("Standard Line",copy_img)        
        if cv.waitKey(1) & 0xFF == ord('q'):  
            break
    return 0

if __name__ == "__main__":
    main(sys.argv[1:])