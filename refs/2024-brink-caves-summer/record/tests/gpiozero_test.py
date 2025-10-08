import gpiozero
from time import sleep
from signal import pause

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

red = gpiozero.LED(red_pin, active_high=False)
yellow = gpiozero.LED(yellow_pin, active_high=False)
blue = gpiozero.LED(blue_pin, active_high=False)
b1 = gpiozero.Button(b1_pin, pull_up=None, active_state=True, bounce_time=0.1)
b2 = gpiozero.Button(b2_pin, pull_up=None, active_state=True, bounce_time=0.1)
b3 = gpiozero.Button(b3_pin, pull_up=None, active_state=True, bounce_time=0.1)
char = gpiozero.LEDCharDisplay(a_pin, b_pin, c_pin, d_pin, e_pin, f_pin, g_pin, active_high=False)

hex_str = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F']

def press1():
	print("button 1 pressed")
	red.toggle()

def press2():
	print("button 2 pressed")
	yellow.toggle()

def press3():
	print("button 3 pressed")
	blue.toggle()

def test_buttons():
	b1.when_pressed = press1
	b2.when_pressed = press2
	b3.when_pressed = press3
	
def main():
	test_buttons()
	while True:
		for c in range(16):
			char.value = hex_str[c]
			sleep(1)
	pause()

main()
