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
send = str(count)

while True:
    #print(stm.name)
    stm.open()    
    stm.write(send.encode())
    line = stm.readline()
    time.sleep(0.02)
    print(line.decode()) 
    count += 1
    if(count >20):
        stm.close()
        break
        
