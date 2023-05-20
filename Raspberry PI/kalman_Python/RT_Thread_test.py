import time
import threading
a = 0
def add(self):
    global a
    a += 1
    print(a)

#thread1 = threading.Thread(target=add(a), args=None, kwargs=None)
timer = time.time()
count = 0
while True:
    count += 1
    if(count == 28050000):
        count = 0
        cnt = time.time()-timer
        timer = time.time()
        time.sleep(1)
        print(cnt)
