#!/usr/bin/env python3

from stretch_body.stepper import Stepper
from stretch_body.pimu import Pimu
from stretch_body.wacc import Wacc
import threading
import time
import os


n = 20
sleep = 0.1
#This script establishes communications with the Pimu at launch then shutsdown
#This has the effect of letting the Pimu know that Ubuntu Desktop is live (which drives its LightBar state machine)
def ping_pimu():
    print('='*80)
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


hello_motor_devices = ['/dev/hello-motor-arm',
                       '/dev/hello-motor-lift',
                       '/dev/hello-motor-left-wheel',
                       '/dev/hello-motor-right-wheel']

def ping_stepper_motors(usb):
    print('='*80)
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

def ping_steppers():
    for dev in hello_motor_devices:
        ping_stepper_motors(dev)

def ping_wacc():
    print('='*80)
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


def timeout(e, timeout, start):
    s = time.time()
    while True:
        if time.time() - s > timeout:
            print(f"Time out: {time.time()-start}")
            os._exit(1)
        if e.is_set():
            os._exit(1)

# The pings on hello devices runs with 15s timeout condition
# The timeout condition is monitered by a thread
start = time.time()
e = threading.Event()
t = threading.Thread(target=timeout,args=(e,15,start))
t.start()
ping_pimu()
ping_wacc()
ping_steppers()
e.set()
print(f"Time elapsed: {time.time()-start}")
        
    

    
            