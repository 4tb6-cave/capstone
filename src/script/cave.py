"""
cave.py - system program to manage CAVE mapping device
Runs on boot to handle GPIO interactions (buttons, LEDs) and manage the status of the system, 
mainly starting and stopping the capture process in ROS.
Nicholas Trimble, 25/02/2026
"""

open("/home/mscp/run.txt", "w") #indicate script has run
import os
import time
import subprocess
import gpiozero as GPIO
import docker

#hardware constants
GPIO_RECORD_TOGGLE = 17     #Button 1 aka U6 - LED U7 linked via hardware
GPIO_SHUTDOWN      = 4      #Button 2 aka U8 - LED U9 linked via hardware
GPIO_RECORD_STATUS = 23     #Sensing aka U10
GPIO_RECORD_ERROR  = 24     #Sensing Error aka LED1
GPIO_STORAGE_IND   = 27     #Battery Low aka LED2               #this may not be working?

#system constants
C_START_STOP = False        #true=running, false=not running
C_START_STOP_D = False      #previous state to detect change- this is very state-machine-y
C_SHUTDOWN = False          #True indicates the Pi will shut down on the next cycle
C_STORAGE_THRES = 0.8       #fractional occupied space where warning begins
C_DEBOUNCE = 20             #Button debouncing time, in ms
C_LOOP_TIME = 0.1           #Loop delay, seconds

#-#-# functions #-#-#

"""
Button callbacks
Input: None, called on ISR
Output: Changed system state
"""
def shutdown_isr(): #gpiozero has mechanics for button press time; we should use this to require a long press to shut down
    global C_SHUTDOWN
    C_SHUTDOWN = True
    print(f"Shutdown ISR callback: Shutdown {C_SHUTDOWN}")

def rec_toggle_isr():
    global C_START_STOP
    C_START_STOP = not C_START_STOP
    print(f"Recording ISR callback: Start/stop {C_START_STOP}")

"""
Determines whether the storage device is almost full as defined by C_STORAGE_THRES
Inputs: none
Outputs: Boolean, True indicates storage has reached the threshold of being 'almost full'
"""
def check_capacity():
    #2 lines from: https://stackoverflow.com/questions/44182042/python-script-to-monitor-disk-space-from-df-and-send-e-mail-alert-when-over-thre
    fs = os.statvfs("/")
    storage_frac =  round((((fs.f_blocks - fs.f_bfree) * fs.f_frsize)/(fs.f_blocks * fs.f_bsize)), 2)
    #print(storage_frac)

    if storage_frac >= C_STORAGE_THRES:
        return True
    else:
        return False
    
#-#-# initialization #-#-#

#hardware objects
but_record_toggle = GPIO.Button(GPIO_RECORD_TOGGLE)
but_shutdown      = GPIO.Button(GPIO_SHUTDOWN)
led_record_status = GPIO.LED(GPIO_RECORD_STATUS)
led_record_error  = GPIO.LED(GPIO_RECORD_ERROR)
led_storage_ind   = GPIO.LED(GPIO_STORAGE_IND)

#button interrupt events
but_shutdown.hold_time = 3
but_shutdown.when_held = shutdown_isr
but_record_toggle.when_pressed = rec_toggle_isr

#-#-# main loop #-#-#

print("Begin process")
while C_SHUTDOWN == False:
    #print(f"Past: {C_START_STOP_D}, Present: {C_START_STOP}")

    #check storage
    if check_capacity() == True:
        led_record_error.on() #temporary
        led_storage_ind.on() #check wiring, this LED isnt turning on

    #'Rising edge', begin recording
    if C_START_STOP == True and C_START_STOP_D == False:
        # start the docker container for recording
        try:
            subprocess.run(["docker", "compose", "up", "-d", "tof"], cwd="/home/mscp/capstone/src", check=True)
            print("Started TOF recording container")
            led_record_status.on()
        except subprocess.CalledProcessError as e:
            print(f"Failed to start container: {e}")
            led_record_error.on()

    #'Falling edge', stop recording
    elif C_START_STOP == False and C_START_STOP_D == True:
        led_record_error.off() #turn off leds, no longer trying to record
        led_record_status.off()
        # stop the docker container
        try:
            subprocess.run(["docker", "compose", "down"], cwd="/home/mscp/capstone/src", check=True)
            print("Stopped recording container")
        except subprocess.CalledProcessError as e:
            print(f"Failed to stop container: {e}")
        os.sync()

    #update state
    C_START_STOP_D = C_START_STOP

    #loop delay
    time.sleep(C_LOOP_TIME)

#loop broken, time to shut down!
os.sync()
for s in range(5,0,-1):
    print(s)
    time.sleep(1)
os.system("systemctl poweroff") #this command may need sudo priveledges, also turning it back on requires powercycling
