#!/usr/bin/env python3

from stretch_body.stepper import Stepper
from stretch_body.pimu import Pimu
from stretch_body.wacc import Wacc
import time


#This script establishes communications with the Pimu at launch then shutsdown
#This has the effect of letting the Pimu know that Ubuntu Desktop is live (which drives its LightBar state machine)
def ping_pimu():
    print('='*80)
    print(f"Starting up Ping on Pimu [{time.asctime()}]")
    for i in range(10):
        try:
            s = Pimu()
            if s.startup():
                print(f"Attempt {i} Ping to Pimu SUCCESS")
                s.stop()
                break
        except Exception as e:
            print(e)
        print(f"Attempt {i} Ping to Pimu FAIL")


hello_motor_devices = ['/dev/hello-motor-arm',
                       '/dev/hello-motor-lift',
                       '/dev/hello-motor-left-wheel',
                       '/dev/hello-motor-right-wheel']

def ping_stepper_motors(usb):
    print('='*80)
    print(f"Starting up Ping on stepper: {usb} [{time.asctime()}]")
    for i in range(10):
        try:
            s = Stepper(usb)
            if s.startup():
                print(f"Attempt {i} Ping to Stepper {usb} SUCCESS")
                s.stop()
                break
        except Exception as e:
            print(e)
    print(f"Attempt {i} Ping to Stepper {usb} FAIL")

def ping_steppers():
    for dev in hello_motor_devices:
        ping_stepper_motors(dev)

def ping_wacc():
    print('='*80)
    print(f"Starting up Ping on Wacc [{time.asctime()}]")
    for i in range(10):
        try:
            s = Wacc()
            if s.startup():
                print(f"Attempt {i} Ping to Wacc SUCCESS")
                s.stop()
                break
        except Exception as e:
            print(e)
    print(f"Attempt {i} Ping to Wacc FAIL")


#The below methods tries to communicate with the Pimu, Wacc and Stepper motors at launch allowing the WDT feature of FW to reset if in bad state
ping_pimu()
ping_wacc()
ping_steppers()

    
            