import threading
import time

def func():
    print('ran')
    time.sleep(1)
    print('done')
    time.sleep(1)
    print('now done')

# x = threading.Thread(target=func, args=(4,))
x = threading.Thread(target=func)
x.start()
print(threading.active_count())
time.sleep(1)
print('finally')