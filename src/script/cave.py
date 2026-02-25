"""
cave.py - system program to manage CAVE mapping device
Runs on boot to handle GPIO interactions (buttons, LEDs) and manage the status of the system, 
mainly starting and stopping the capture process in ROS.
Nicholas Trimble, 25/02/2026
"""
import os
import time
import gpiozero as GPIO

#hardware constants
GPIO_RECORD_TOGGLE = 17     #Button 1 aka U6 - LED U7 linked via hardware
GPIO_SHUTDOWN      = 4      #Button 2 aka U8 - LED U9 linked via hardware
GPIO_RECORD_STATUS = 23     #Sensing aka U10
GPIO_RECORD_ERROR  = 24     #Sensing Error aka LED1
GPIO_BATTERY_IND   = 27     #Battery Low aka LED2

#hardware objects
but_record_toggle = GPIO.Button(GPIO_RECORD_TOGGLE)
but_shutdown      = GPIO.Button(GPIO_SHUTDOWN)
led_record_status = GPIO.LED(GPIO_RECORD_STATUS)
led_record_error  = GPIO.LED(GPIO_RECORD_ERROR)
led_battery_ind   = GPIO.LED(GPIO_BATTERY_IND)

#system constants
C_START_STOP = False        #true=running, false=not running
C_STORAGE_THRES = 0.8       #fractional occupied space where warning begins

#startup

"""
Determines whether the storage device is almost full as defined by C_STORAGE_THRES
Inputs: none
Outputs: Boolean, True indicates storage has reached the threshold of being 'almost full'
"""
def check_capacity():
    storage_cap  = 2 #some os call
    storage_used = 1 #some os call
    storage_frac = storage_used / storage_cap
    
    if storage_frac >= C_STORAGE_THRES:
        return True
    else
        return False

#monitoring loop
while not but_shutdown.is_pressed:

    #check storage
    if 

    #Begin recording
    if but_record_toggle.is_pressed and not C_START_STOP:
        C_START_STOP = True
        #exec the capture script asynchronously

    #End recording
    else if but_record_toggle.is_pressed and C_START_STOP:
        C_START_STOP = False
        #
    else:

    time.sleep(100)
