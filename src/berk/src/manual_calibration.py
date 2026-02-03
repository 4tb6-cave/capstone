import board
import busio
import adafruit_bno055
from adafruit_bno055 import BNO055

try:
    i2c = board.I2C()  # uses board.SCL and board.SDA
    sensor = adafruit_bno055.BNO055_I2C(i2c)
except OSError as e:
    print("\n❌ ERROR: Could not connect to BNO055!")
    print("Check wiring:")
    print("- Ensure SCL/SDA are connected correctly")
    print("- Verify 3.3V power is applied")
    print("- Check if sensor has soldered headers (if using breakout board)")
    exit(1)

print("="*70)
print("BNO055 MANUAL CALIBRATION DATA LOADER")
print("="*70)
print("\n⚠️ WARNING: Using incorrect calibration values can DEGRADE sensor performance!")
print("- Only use this if you have COMPLETE and ACCURATE calibration data from a reliable source")
print("- Typical radius values should NOT be guessed - use calibrated data instead")
print("- If unsure, run the standard auto-calibration process (use original script)")
print("\nNote: BNO055 requires BOTH offset AND radius values for magnetometer/accelerometer\n")

# Current calibration for reference
#current_cal = sensor.get_calibration()
#if current_cal:
#    print("Current calibration data (for comparison):")
#    print(f"  {current_cal}\n")

def get_signed_int(prompt, default=None):
    while True:
        try:
            user_input = input(prompt)
            if user_input == '' and default is not None:
                return default
            value = int(user_input)
            # Verify it fits in 16-bit signed integer range (-32768 to 32767)
            if -32768 <= value <= 32767:
                return value
            print("ERROR: Value must be between -32768 and 32767")
        except ValueError:
            print("ERROR: Please enter a valid integer")

# Get magnetometer data (offsets + radius)
print("\nMAGNETOMETER DATA (REQUIRED):")
mag_x = get_signed_int("  Magnetometer X offset (-500 to 500 typical): ")
mag_y = get_signed_int("  Magnetometer Y offset: ")
mag_z = get_signed_int("  Magnetometer Z offset: ")
mag_radius = get_signed_int("  Magnetometer radius (2400 ± 600 typically): ", 2400)

# Get accelerometer data (offsets + radius)
print("\nACCELEROMETER DATA (REQUIRED):")
acc_x = get_signed_int("  Accelerometer X offset (-100 to 100 typical): ")
acc_y = get_signed_int("  Accelerometer Y offset: ")
acc_z = get_signed_int("  Accelerometer Z offset: ")
acc_radius = get_signed_int("  Accelerometer radius (9830 ± 50 typically for 1g): ", 9830)

# Get gyroscope data (offsets only)
print("\nGYROSCOPE DATA (REQUIRED):")
gyro_x = get_signed_int("  Gyroscope X offset (-10 to 10 typical): ")
gyro_y = get_signed_int("  Gyroscope Y offset: ")
gyro_z = get_signed_int("  Gyroscope Z offset: ")

# Convert integers to two-byte LSB-first format
def int_to_bytes(value):
    return [value & 0xFF, (value >> 8) & 0xFF]

cal_data = []
# Magnetometer offsets + radius
cal_data.extend(int_to_bytes(mag_x))
cal_data.extend(int_to_bytes(mag_y))
cal_data.extend(int_to_bytes(mag_z))
cal_data.extend(int_to_bytes(mag_radius))

# Accelerometer offsets + radius
cal_data.extend(int_to_bytes(acc_x))
cal_data.extend(int_to_bytes(acc_y))
cal_data.extend(int_to_bytes(acc_z))
cal_data.extend(int_to_bytes(acc_radius))

# Gyroscope offsets only (no radius)
cal_data.extend(int_to_bytes(gyro_x))
cal_data.extend(int_to_bytes(gyro_y))
cal_data.extend(int_to_bytes(gyro_z))

print("\n" + "="*70)
print("CALIBRATION DATA PREVIEW:")
for i in range(0, len(cal_data), 2):
    byte_val = cal_data[i] | (cal_data[i+1] << 8)
    # Convert to signed integer for display
    if byte_val > 32767:
        actual_val = byte_val - 65536
    else:
        actual_val = byte_val
    print(f"Bytes [{i:02d}-{i+1:02d}]: {actual_val} (Hex: {cal_data[i]:#x}, {cal_data[i+1]:#x})")

print("\nWARNING: This data will overwrite current calibration!")
confirm = input("Write these values to sensor? (Y/n): ").strip().lower()
if confirm == 'y' or confirm == '':
    try:
        sensor.set_calibration(cal_data)
        print("\n✅ Calibration data written successfully!")
        
        # Verify write
        new_cal = sensor.get_calibration()
        if new_cal == cal_data:
            print("✓ Verification: Data matches what was written")
        else:
            print("! Warning: Readback data doesn't match written values (check wiring)")
    except Exception as e:
        print(f"❌ Error writing calibration: {e}")
else:
    print("\n❌ Operation cancelled. No changes made.")
