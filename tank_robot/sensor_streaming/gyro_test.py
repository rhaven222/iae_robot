from mpu6050 import mpu6050

sensor = mpu6050(0x68)

print(sensor.get_gyro_data())
print(sensor.get_accel_data())