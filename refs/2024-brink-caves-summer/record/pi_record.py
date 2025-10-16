import gpiozero
import time
from signal import pause
from oak_driver.oak_driver import OakDriver
import psutil
import os

b1_pin = 5
b2_pin = 13
b3_pin = 26

red_pin = 6
yellow_pin = 12
blue_pin = 19

a_pin = 24
b_pin = 7
c_pin = 8
d_pin = 11
e_pin = 25
f_pin = 10
g_pin = 9

hex_str = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F']

b_factor = 1 / (2 ** 20) # to convert bytes to megabytes

def hex_display_bandwidth(b):
	i = int(b * b_factor * 0.1)
	if i < 0:
		return '0' # prevent negative numbers
	elif i > 10:
		return 'O' # overload
	else:
		return hex_str[i]

def main():

	red = gpiozero.PWMLED(red_pin, active_high=False)
	yellow = gpiozero.PWMLED(yellow_pin, active_high=False)
	blue = gpiozero.PWMLED(blue_pin, active_high=False)
	b1 = gpiozero.Button(b1_pin, pull_up=None, active_state=True, bounce_time=0.1)
	b2 = gpiozero.Button(b2_pin, pull_up=None, active_state=True, bounce_time=0.1)
	b3 = gpiozero.Button(b3_pin, pull_up=None, active_state=True, bounce_time=0.1, hold_time=1)
	char = gpiozero.LEDCharDisplay(a_pin, b_pin, c_pin, d_pin, e_pin, f_pin, g_pin, active_high=False)

	d = OakDriver(baseDir="/home/slam/recordings")

	config_n = 0
	config_dir = '/home/slam/config'

	def set_config(directory, number):
		char.value = hex_str[number]
		file = os.path.join(directory, f'{number:02d}.yaml')
		if os.path.exists(file):
			d.set_config_file(file)
		else:
			d.set_config_dict({})

	set_config(config_dir, config_n)

	def press1():
		print("Button 1 pressed: start recording")
		success = d.start_record()
		if (not success):
			print("failed to start recording")

	def press2():
		print("Button 2 pressed: stop recording")
		success = d.stop_record()
		if (not success):
			print("failed to stop recording")

	def press3():
		print("button 3 pressed: increment hex")
		nonlocal config_n
		if (d.get_recording_status() == d.RECORD_OFF):
			config_n = (config_n + 1) % 16
			set_config(config_dir, config_n)

	def hold3():
		print("button 3 held: reset hex")
		nonlocal config_n
		if (d.get_recording_status() == d.RECORD_OFF):
			config_n = 0
			set_config(config_dir, config_n)

	b1.when_pressed = press1
	b2.when_pressed = press2
	b3.when_pressed = press3
	b3.when_held = hold3

	disk_usage_old = psutil.disk_usage('/').used

	red_intensity = 0

	while True:
		for i in range(10):
			start_time = time.time()

			match (d.get_recording_status()):
				case d.RECORD_OK:
					red_intensity = 1
				case d.RECORD_START:
					red_intensity = (red_intensity + 0.05) % 1
				case d.RECORD_STOP:
					red_intensity = (red_intensity - 0.05) % 1
				case d.RECORD_OFF:
					red_intensity = 0
				case _:
					red_intensity = (red_intensity + 0.2) % 1

			red.value = red_intensity ** 2

			cpu = psutil.cpu_percent()
			disk = psutil.disk_usage('/')
			disk_usage_new = disk.used
			bandwidth = (disk_usage_new - disk_usage_old) * 10 # bytes per second
			disk_usage_old = disk_usage_new
			# print("memory write speed:", bandwidth * b_factor, "MiB/s")
			
			yellow.value = cpu / 100

			if disk.percent > i*10:
				blue.on()
			else:
				blue.off()

			if (d.get_recording_status() != d.RECORD_OFF):
				char.value = hex_display_bandwidth(bandwidth)
			else:
				char.value = hex_str[config_n]

			time.sleep(0.1 - (time.time() - start_time)) # sleep so each loop is exactly 0.1 seconds
	
if __name__=="__main__":
	
	main()
