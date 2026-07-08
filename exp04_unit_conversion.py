# exp04_unit_conversion.py
# 实验四：物理单位转换
# 将原始值转换为 g 和 deg/s
# 水平放置确认 az 约 1.0g
# 绕 Z 轴旋转 90 度积分验证
#
# 转换系数 (PPT 第8页):
#   加速度 (±2g):  16384 LSB/g
#   陀螺仪 (±250deg/s): 131 LSB/deg/s
# ============================================

from machine import I2C, Pin
from time import sleep
import math

MPU6050_ADDR = 0x68
ACCEL_SCALE = 16384.0   # LSB/g
GYRO_SCALE  = 131.0     # LSB/deg/s

i2c = I2C(0, scl=Pin(21), sda=Pin(47), freq=400000)
i2c.writeto_mem(MPU6050_ADDR, 0x6B, b'\x00')
sleep(0.1)


def combine(h, l):
    val = (h << 8) | l
    if val >= 0x8000:
        val -= 0x10000
    return val


def read_mpu6050():
    data = i2c.readfrom_mem(MPU6050_ADDR, 0x3B, 14)
    ax = combine(data[0], data[1])
    ay = combine(data[2], data[3])
    az = combine(data[4], data[5])
    gx = combine(data[8],  data[9])
    gy = combine(data[10], data[11])
    gz = combine(data[12], data[13])
    return ax, ay, az, gx, gy, gz


# 实验三标定值 (如果已执行实验三，替换为实际值)
# GYRO_BIAS = (0, 0, 0)
GYRO_BIAS = (482, 137, -3066)   # 替换为 calibrate_gyro() 的结果


print("=" * 50)
print("实验四：物理单位转换")
print("=" * 50)
print("加速度格式: ax(g), ay(g), az(g)")
print("陀螺仪格式: gx(deg/s), gy(deg/s), gz(deg/s)")
print("Z角度: 陀螺仪 Z 轴角速度积分 (deg)")
print("---")
print("验证方法:")
print("  1. 水平静止 -> az 约 1.0g")
print("  2. 绕 Z 轴旋转 90 度 -> Z角度 约 90 deg")
print("  3. 前后加速 -> ax 变化")
print("---")
print("倾斜角计算 (PPT 第12页):")
print("  倾斜角 = acos(az/16384) * 180 / pi")
print("---")

try:
    last_time = 0
    angle_z = 0.0

    while True:
        ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw = read_mpu6050()

        # 物理单位转换
        ax_g = ax_raw / ACCEL_SCALE
        ay_g = ay_raw / ACCEL_SCALE
        az_g = az_raw / ACCEL_SCALE

        gx_dps = (gx_raw - GYRO_BIAS[0]) / GYRO_SCALE
        gy_dps = (gy_raw - GYRO_BIAS[1]) / GYRO_SCALE
        gz_dps = (gz_raw - GYRO_BIAS[2]) / GYRO_SCALE

        # 时间差计算 (用于积分)
        now = 0  # 实际用 time.ticks_ms()
        from time import ticks_ms, ticks_diff
        now = ticks_ms()
        if last_time == 0:
            dt = 0.1
        else:
            dt = ticks_diff(now, last_time) / 1000.0
        last_time = now

        # Z 轴角度积分 (绕 Z 轴旋转的总角度)
        angle_z += gz_dps * dt
        if angle_z > 360 or angle_z < -360:
            angle_z = 0   # 防止溢出，超出 360 度自动重置

        # 倾斜角 (与水平面的夹角, PPT 第12页)
        tilt = math.acos(max(-1, min(1, az_raw / ACCEL_SCALE))) * 180 / math.pi

        txt = "Acc(g):{:6.3f},{:6.3f},{:6.3f} | Gyro(deg/s):{:7.2f},{:7.2f},{:7.2f}"
        print(txt.format(ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps))
        print("  Z角度:{:7.1f}deg  倾斜角:{:5.1f}deg".format(angle_z, tilt))
        print("  (验证: 水平静止时 az=1.0g, 倾斜角=0deg)")
        print("  (绕 Z 轴转 90 度看 Z 角度)")

        sleep(0.1)

except KeyboardInterrupt:
    print("\n实验结束")
    print("建议: 结合实验三的零偏值填入 GYRO_BIAS 会得到更准确的结果")
