import time
import serial

stm = serial.Serial(port= '/dev/ttyAMA1',baudrate=19200,timeout=1.0,
                    bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE)
time.sleep(2)

num = 1
while True:
    power = "Send "+str(num)
    stm.reset_output_buffer()
    stm.write(power.encode(encoding='UTF-8',errors='ignore'))
    
    time.sleep(0.03)
    
    num+=1
    data= stm.read(20)
    print(data.decode(encoding='utf-8',errors='ignore'))
    time.sleep(0.03)