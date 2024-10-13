import smbus
import time

bus = smbus.SMBus(1)  # I2C1

# Define the I2C addresses for the four sensors
sensor_1_address = 0x54  # Replace with actual I2C address of sensor 1

# Function to read a byte from a sensor
def read_sensor_data(address, register):
    try:
        data = bus.read_byte_data(address, register)
        return data
    except Exception as e:
        print(f"Error reading from sensor at address {hex(address)}: {e}")
        return None

# Define the register you want to read from
# This depends on the sensor type. Adjust accordingly.
sensor_register = 0x00  # Replace with actual register for your sensor

# Main loop to read from the sensors
try:
    while True:
        # Read data from sensor 1
        sensor_1_data = read_sensor_data(sensor_1_address, sensor_register)
        print(f"Sensor 1 data: {sensor_1_data}")
        
        # Wait for a short period before the next read
        time.sleep(1)

except KeyboardInterrupt:
    print("Stopping the sensor read operation.")
