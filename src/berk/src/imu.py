# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import board
import adafruit_bno055
import json  # Added for JSON serialization

i2c = board.I2C()  # uses board.SCL and board.SDA
sensor = adafruit_bno055.BNO055_I2C(i2c)

last_val = 0xFFFF

def temperature():
    global last_val
    result = sensor.temperature
    if abs(result - last_val) == 128:
        result = sensor.temperature
        if abs(result - last_val) == 128:
            return 0b00111111 & result
    last_val = result
    return result

with open('bno055_data.json', 'w') as f:
    while True:
        # Collect all sensor data into a dictionary with timestamps
        data = {
            "timestamp": time.time(),
            "temperature": temperature(),
            "acceleration": list(sensor.acceleration),
            "magnetic": list(sensor.magnetic),
            "gyro": list(sensor.gyro),
            "euler": list(sensor.euler),
            "quaternion": list(sensor.quaternion),
            "linear_acceleration": list(sensor.linear_acceleration),
            "gravity": list(sensor.gravity)
        }

        # Append each reading as a JSON line to the file
        json.dump(data, f)
        f.write('\n')
    
#    time.sleep(0.01)
