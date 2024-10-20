import smbus
import time
import RPi.GPIO as GPIO

# Initialize the I2C bus (bus 1 on Raspberry Pi)
bus = smbus.SMBus(1)

# Define the I2C address for the VL6180X sensor
sensor_1_address = 0x29  # VL6180X default I2C address

# Setup GPIO for PWM (to control LED)
LED_PIN = 18  # Use GPIO18 for PWM (Pin 12 on the Raspberry Pi header)
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)

# Set up PWM on the LED pin with a frequency of 1000 Hz
pwm = GPIO.PWM(LED_PIN, 1000)
pwm.start(0)  # Start PWM with 0% duty cycle (LED off)

# Function to read a byte from a sensor
def read_sensor_data(address, register):
    try:
        data = bus.read_byte_data(address, register)
        return data
    except Exception as e:
        print(f"Error reading from sensor at address {hex(address)}: {e}")
        return None

# Define the register you want to read from (proximity or range register)
sensor_register = 0x04  # Register for proximity data (depending on the sensor's datasheet)

# Function to scale sensor data to a PWM duty cycle
def scale_sensor_output_to_pwm(sensor_value):
    max_sensor_value = 255  # Adjust based on your sensor's output range
    duty_cycle = (sensor_value / max_sensor_value) * 100
    return min(max(duty_cycle, 0), 100)  # Ensure it's between 0 and 100

# Main loop to read from the sensors and control LED brightness
try:
    while True:
        # Read data from the VL6180X sensor
        sensor_1_data = read_sensor_data(sensor_1_address, sensor_register)
        if sensor_1_data is not None:
            print(f"Sensor 1 data: {sensor_1_data}")
            
            # Scale sensor output to a PWM duty cycle
            pwm_value = scale_sensor_output_to_pwm(sensor_1_data)
            
            # Update the PWM duty cycle to control the LED brightness
            pwm.ChangeDutyCycle(pwm_value)
            print(f"PWM Duty Cycle: {pwm_value}%")
        
        # Wait for a short period before the next read
        time.sleep(1)

except KeyboardInterrupt:
    print("Stopping the sensor read operation.")
    pwm.stop()  # Stop PWM
    GPIO.cleanup()  # Clean up GPIO settings
