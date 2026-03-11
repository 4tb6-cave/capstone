"""
cave.py - system program to manage CAVE mapping device
Runs on boot to handle GPIO interactions (buttons, LEDs) and manage the status of the system, 
mainly starting and stopping the capture process in ROS.
Nicholas Trimble, 25/02/2026
"""

import os
import sys
import time
import gpiozero as GPIO
import subprocess

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
C_LOOP_TIME = 0.1           #Loop delay, seconds
C_WORKDIR   = "/etc/cave/src" #location of sources
C_DOCKER_BASE = ["docker", "compose", "-f", f"{C_WORKDIR}/record-compose.yml"]
C_SEEN_LOG_LINES = set()    #Cache of seen lines (bounded to prevent memory growth)

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

def print_subprocess_output(result):
    """Print stdout and stderr from a subprocess result"""
    if result.stdout:
        print(result.stdout, end='')
    if result.stderr:
        print(result.stderr, file=sys.stderr, end='')

def stream_docker_logs():
    """Fetch and print new docker logs, avoiding duplicates with cache"""
    global C_SEEN_LOG_LINES
    try:
        # Use tail to get recent logs
        result = subprocess.run(C_DOCKER_BASE + ["logs", "--tail", "50"],
                                cwd=C_WORKDIR,
                                capture_output=True,
                                text=True,
                                timeout=3
        )
        if result.stdout:
            lines = result.stdout.splitlines()
            # Print only lines we haven't seen before
            for line in lines:
                if line not in C_SEEN_LOG_LINES:
                    print(line)
                    C_SEEN_LOG_LINES.add(line)
            
            # Keep cache bounded - clear if it grows too large
            if len(C_SEEN_LOG_LINES) > 50:
                C_SEEN_LOG_LINES.clear()
    except subprocess.TimeoutExpired:
        pass
    except subprocess.CalledProcessError:
        pass
    
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
    else:
        led_storage_ind.off() #check wiring, this LED isnt turning on


    #stream logs while recording
    if C_START_STOP == True:
        stream_docker_logs()

    #'Rising edge', begin recording
    if C_START_STOP == True and C_START_STOP_D == False:
        # start the docker container for recording
        C_SEEN_LOG_LINES.clear()  # Clear log cache when starting
        try:
            result = subprocess.run(C_DOCKER_BASE + ["up", "-d"],
                                    cwd=C_WORKDIR,
                                    check=True,
                                    capture_output=True,
                                    text=True)
            print_subprocess_output(result)
            print("Started TOF recording container")
            led_record_status.on()
        except subprocess.CalledProcessError as e:
            print(f"Failed to start container: {e}")
            led_record_error.on()

    #'Falling edge', stop recording
    elif C_START_STOP == False and C_START_STOP_D == True:
        # stop the docker container
        try:
            result = subprocess.run(C_DOCKER_BASE + ["down"],
                                    cwd=C_WORKDIR,
                                    check=True,
                                    capture_output=True,
                                    text=True)
            print_subprocess_output(result)
            print("Stopped recording container")
            led_record_error.off() #turn off leds, no longer trying to record
            led_record_status.off()
        except subprocess.CalledProcessError as e:
            print(f"Failed to stop container: {e}")
        C_SEEN_LOG_LINES.clear()  # Clear log cache when stopping
        os.sync()

    #update state
    C_START_STOP_D = C_START_STOP

    #loop delay
    time.sleep(C_LOOP_TIME)

#loop broken, time to shut down!
#bring down container
try:
    result = subprocess.run(C_DOCKER_BASE + ["down"],
                            cwd=C_WORKDIR,
                            check=True,
                            capture_output=True,
                            text=True)
    print_subprocess_output(result)
    print("Stopped recording container")
except subprocess.CalledProcessError as e:
    print(f"Failed to stop container: {e}")
#sync files
os.sync()
#release gpio
but_record_toggle.close()
but_shutdown.close()
led_record_status.close()
led_record_error.close()
led_storage_ind.close()
#countdown
for s in range(5,0,-1):
    print(s)
    time.sleep(1)
os.system("systemctl poweroff") #turning back on will require a power cycle
