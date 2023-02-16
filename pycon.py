import serial
import time
import threading


stm = serial.Serial(port='COM6',
                    baudrate=9600,
                    timeout=1,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE)
line= ''

"""while True:
    
"""
count = 1
while True:
    send = str(count)
    stm.write(send.encode())    
    time.sleep(0.02)
    line = stm.readline()
    time.sleep(0.02)
    #line_left= stm.inWaiting()
    #line += stm.readline(line_left)
    print(line.decode()) 
    count += 1
    if(count >20):
        break
