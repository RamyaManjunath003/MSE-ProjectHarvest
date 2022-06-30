import threading 
import time

class myThread(threading.Thread):
    def __init__(self, threadId, name, count):
        threading.Thread.__init__(self)
        self.threadId = threadId
        self.name = name
        self.count = count
        
    def run(self):
        print('starting: ' + self.name + '\n')

        print_time(self.name, 1, self.count)

        print('Exiting: ' + self.name + '\n')

def print_time(name, delay, count):
    while count:
        time.sleep(delay)
        print('%s: %s %s' % (name, time.ctime(time.time()), count) + '\n')
        count -= 1

thread1 = myThread(1, 'Thread1', 5)
thread2 = myThread(1, 'Thread2', 5)

thread1.start()
thread2.start()
thread1.join()
thread2.join()
print('Done main thread')