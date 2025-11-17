import smbus2
import time

MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B

ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

bus = smbus2.SMBus(1)   # Jetson Orin Nano uses I2C bus 1

def read_word(reg):
    high = bus.read_byte_data(MPU6050_ADDR, reg)
    low = bus.read_byte_data(MPU6050_ADDR, reg + 1)
    value = (high << 8) + low
    if value >= 0x8000:
        value -= 65536
    return value

def init_mpu():
    # Wake up MPU6050 (it starts in sleep mode)
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
    time.sleep(0.1)

def read_accel_gyro():
    ax = read_word(ACCEL_XOUT_H) / 16384.0   # g
    ay = read_word(ACCEL_XOUT_H + 2) / 16384.0
    az = read_word(ACCEL_XOUT_H + 4) / 16384.0

    gx = read_word(GYRO_XOUT_H) / 131.0      # deg/sec
    gy = read_word(GYRO_XOUT_H + 2) / 131.0
    gz = read_word(GYRO_XOUT_H + 4) / 131.0

    return ax, ay, az, gx, gy, gz

if __name__ == "__main__":
    init_mpu()
    print("MPU6050 initialized. Reading data...\n")

    while True:
        ax, ay, az, gx, gy, gz = read_accel_gyro()
        print(f"Accel: ax={ax:.2f}g ay={ay:.2f}g az={az:.2f}g | "
              f"Gyro: gx={gx:.2f}°/s gy={gy:.2f}°/s gz={gz:.2f}°/s")
        time.sleep(0.5)
