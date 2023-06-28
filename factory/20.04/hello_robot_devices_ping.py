#!/usr/bin/env python3

from stretch_body.stepper import Stepper
from stretch_body.pimu import Pimu
from stretch_body.wacc import Wacc
import multiprocessing
import time

class TimeoutProcesses:
    """
    Execute each given list of worker functions with arguments from a different process
    and kill all the process based on a timeout
    
    timeout_s: Timout in seconds
    worker_functiosn: [] of tuples containing the worker functions with arguments
                      E.g. [(func, arg1, arg2), ...] 
    """
    def __init__(self, timeout_s, worker_functions=[]):
        self.timeout = timeout_s
        self.worker_functions = worker_functions
        self.procs = []
        self.success = True 
    
    def populate_procs(self):
        if len(self.worker_functions)>0:
            for i in range(len(self.worker_functions)):
                target = self.worker_functions[i][0]
                if len(self.worker_functions[i][1:])>=1:
                   args =  self.worker_functions[i][1:]
                else:
                    args = ()
                p = multiprocessing.Process(target=target, 
                                            args=args, 
                                            name = ('process_' + str(i+1)))
                self.procs.append(p)
                p.start()
                print('starting', p.name)
    
    def run(self):
        self.populate_procs()
        start = time.time()
        while time.time() - start <= self.timeout:
            if not any(p.is_alive() for p in self.procs):
                break
            time.sleep(0.1)
        else:
            self.success = False
            print(f"Timed out: {self.timeout}s, killing all processes")
            for p in self.procs:
                p.terminate()
                p.join()        
        print(f"Total time elapsed: {time.time()-start}s")
        return self.success
    
n = 10
sleep = 0.1

#This script establishes communications with the Pimu at launch then shutsdown
#This has the effect of letting the Pimu know that Ubuntu Desktop is live (which drives its LightBar state machine)
def ping_pimu():
    print(f"Starting up Ping on Pimu [{time.asctime()}]")
    for i in range(n):
        time.sleep(sleep)
        try:
            s = Pimu()
            if s.startup():
                print(f"Attempt {i} Ping to Pimu SUCCESS")
                s.stop()
                break
            else:
                print(f"Attempt {i} Ping to Pimu FAIL")
        except Exception as e:
            print(e)

def ping_stepper_motors(usb):
    print(f"Starting up Ping on stepper: {usb} [{time.asctime()}]")
    for i in range(n):
        time.sleep(sleep)
        try:
            s = Stepper(usb)
            if s.startup():
                print(f"Attempt {i} Ping to Stepper {usb} SUCCESS")
                s.stop()
                break
            else:
                print(f"Attempt {i} Ping to Stepper {usb} FAIL")
        except Exception as e:
            print(e)

def ping_wacc():
    print(f"Starting up Ping on Wacc [{time.asctime()}]")
    for i in range(n):
        time.sleep(sleep)
        try:
            s = Wacc()
            if s.startup():
                print(f"Attempt {i} Ping to Wacc SUCCESS")
                s.stop()
                break
            else:
                print(f"Attempt {i} Ping to Wacc FAIL")
        except Exception as e:
            print(e)
            
worker_functions = [(ping_pimu,),
                    (ping_wacc,),
                    (ping_stepper_motors,'/dev/hello-motor-arm'),
                    (ping_stepper_motors,'/dev/hello-motor-lift'),
                    (ping_stepper_motors,'/dev/hello-motor-right-wheel'),
                    (ping_stepper_motors,'/dev/hello-motor-left-wheel')]

tp = TimeoutProcesses(timeout_s=2, worker_functions=worker_functions)
all_devices_successfully_pinged = tp.run()
        
    

    
            