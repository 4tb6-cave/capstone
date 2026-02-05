#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""
imu.py – IMU sensor daemon that listens on a Unix domain socket.
When it receives a message on /record/imu_trigger it records one sample
to bno055_data.json.

Usage:
    # Start the daemon (run as root or a user with write permission to /record)
    python3 imu.py

    # Trigger a recording from another terminal/process
    echo "start" | socat - UNIX-CONNECT:/record/imu_trigger
"""

import time
import board
import adafruit_bno055
import json
import socket
import os
import sys


# ----------------------------------------------------------------------
# 1. Sensor setup (unchanged)
# ----------------------------------------------------------------------
i2c = board.I2C()            # uses board.SCL and board.SDA
sensor = adafruit_bno055.BNO055_I2C(i2c)

last_val = 0xFFFF

def temperature():
    """Return a stable temperature reading."""
    global last_val
    result = sensor.temperature
    if abs(result - last_val) == 128:
        # A known quirk in the BNO055 driver – read again
        result = sensor.temperature
        if abs(result - last_val) == 128:
            return 0b00111111 & result
    last_val = result
    return result


def record_sample(f):
    """
    Collect a single IMU sample and write it as JSON to the file object `f`.
    The file is expected to be opened in append mode.
    """
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
    json.dump(data, f)
    f.write('\n')
    f.flush()


# ----------------------------------------------------------------------
# 2. Unix domain socket server
# ----------------------------------------------------------------------
SOCKET_PATH = '/home/admin/capstone/src/record/imu_trigger'

def cleanup_socket():
    """Remove the socket file if it already exists."""
    try:
        os.unlink(SOCKET_PATH)
    except OSError as e:
        # Ignore “file not found” errors – they’re expected on first run.
        if e.errno != 2:  # errno 2 == No such file or directory
            raise


def start_server():
    """Main loop that listens for triggers and records samples."""
    cleanup_socket()
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(SOCKET_PATH)
    os.chmod(SOCKET_PATH, 0o777)          # make the socket world‑accessible
    server.listen(1)

    print(f"IMU trigger listener started – waiting on {SOCKET_PATH}", file=sys.stderr)

    try:
        while True:
            conn, _ = server.accept()
            with conn:                       # automatically close when done
                buffer = b''
                while True:
                    chunk = conn.recv(1024)
                    if not chunk:          # client closed the connection
                        break

                    buffer += chunk

                    # Treat each newline‑delimited line as a separate trigger.
                    while b'\n' in buffer:
                        line, buffer = buffer.split(b'\n', 1)
                        message = line.decode('utf-8').strip()
                        if message:           # non‑empty message → record
                            with open('bno055_data.json', 'a') as f:
                                record_sample(f)
                            try:
                                conn.sendall(b'OK\n')
                            except BrokenPipeError:
                                pass

                # If the client sent data without a trailing newline, handle it.
                if buffer.strip():
                    with open('bno055_data.json', 'a') as f:
                        record_sample(f)
                print("yummy data")

    finally:
        server.close()
        cleanup_socket()


# ----------------------------------------------------------------------
if __name__ == "__main__":
    start_server()
