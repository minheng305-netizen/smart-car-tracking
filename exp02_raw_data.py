# exp02_raw_data.py
# 实验二：原始数据读取
# 编写 read_mpu6050() 函数，循环打印加速度和角速度原始值，
# 观察模块运动时数值变化
#
# 寄存器映射 (PPT 第8页):
#   0x3B~0x40  加速度 X/Y/Z (6字节)
#   0x41~0x42  温度 (2字节，跳过)
#   0x43~0x48  角速度 X/Y/Z (6字节)
# ============================================

from machine import I2C, Pin
from time import sleep
import math

MPU6050_ADDR = 0x68

i2c = I2C(0, scl=Pin(21), sda=Pin(47), freq=400000)
i2c.writeto_mem(MPU6050_ADDR, 0x6B, b'\x00')
sleep(0.1)


def combine(h, l):
    """将高字节和低字节合并为有符号16位整数 (PPT 第10页)"""
    val = (h << 8) | l
    if val >= 0x8000:
        val -= 0x10000
    return val


def read_mpu6050():
    """读取加速度和角速度原始值，返回 (ax, ay, az, gx, gy, gz)"""
    data = i2c.readfrom_mem(MPU6050_ADDR, 0x3B, 14)
    ax = combine(data[0], data[1])
    ay = combine(data[2], data[3])
    az = combine(data[4], data[5])
    gx = combine(data[8],  data[9])
    gy = combine(data[10], data[11])
    gz = combine(data[12], data[13])
    return ax, ay, az, gx, gy, gz


print("=" * 50)
print("实验二：MPU6050 原始数据读取")
print("=" * 50)
print("静止预期: ax=0, ay=0, az=+16384, gx=0, gy=0, gz=0")
print("试试: ①缓慢倾斜 ②绕Z轴旋转 ③前后加速")
print("-" * 50)

try:
    count = 0
    while True:
        ax, ay, az, gx, gy, gz = read_mpu6050()
        acc_mag = math.sqrt(ax*ax + ay*ay + az*az)
        count += 1
        txt = "[{:4d}] A:{:6d},{:6d},{:6d} | G:{:6d},{:6d},{:6d} | |a|={:7.1f}"
        print(txt.format(count, ax, ay, az, gx, gy, gz, acc_mag))
        sleep(0.1)

except KeyboardInterrupt:
    print("\n读取", count, "组数据，实验结束")
