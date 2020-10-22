import threading
import time
import sys
def runThread(a):
    # lock.acquire()
    print('arguments:',a)
    print(a,'1')
    time.sleep(1)
    print(a,'2')
    time.sleep(1)
    print(a,'3')
    # lock.release()
if __name__ == "__main__":
    lock=threading.Lock()
    for i in range(2):
        thread1=threading.Thread(target=runThread,args=(i,))
        thread1.start()
        thread1.join()
    print('end')