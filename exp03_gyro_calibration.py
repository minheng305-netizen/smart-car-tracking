# exp03_gyro_calibration.py
# 实验三：陀螺仪标定（零偏校准）
# 将模块水平静置，采集 200 组角速度数据，
# 计算零偏值，验证扣除后静止读数约等于 0
#
# 原理 (PPT 第11页):
#   MEMS 陀螺仪出厂有零偏（静止时输出非零）
#   标定 = 采集静止平均值 → 后续读数中扣除
#   每次上电零偏可能微变，建议每次运行前标定
# ============================================

from machine import I2C, Pin
from time import sleep

MPU6050_ADDR = 0x68
CAL_SAMPLES = 200

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


def calibrate_gyro(n=CAL_SAMPLES):
    """标定陀螺仪零偏 (PPT 第11页)
       水平静止，采集 n 组取平均值
       返回: (bias_x, bias_y, bias_z)"""
    print(">>> 正在标定陀螺仪... 请保持模块绝对静止!")
    sum_gx = sum_gy = sum_gz = 0

    for i in range(n):
        _, _, _, gx, gy, gz = read_mpu6050()
        sum_gx += gx
        sum_gy += gy
        sum_gz += gz
        if i % 40 == 0:
            print("  进度: {}%".format((i * 100) // n))
        sleep(0.01)

    bias_x = sum_gx // n
    bias_y = sum_gy // n
    bias_z = sum_gz // n

    print("  标定完成!")
    print("  零偏值 (LSB): X={}, Y={}, Z={}".format(bias_x, bias_y, bias_z))
    print("  物理单位 (°/s): X={:.2f}, Y={:.2f}, Z={:.2f}".format(
        bias_x / 131.0, bias_y / 131.0, bias_z / 131.0))
    return bias_x, bias_y, bias_z


def get_corrected_gyro(bias):
    """读取陀螺仪数据并扣除零偏
       返回: (gx_cal, gy_cal, gz_cal)"""
    _, _, _, gx, gy, gz = read_mpu6050()
    return gx - bias[0], gy - bias[1], gz - bias[2]


# ============================================
print("=" * 50)
print("实验三：陀螺仪标定")
print("=" * 50)

GYRO_BIAS = calibrate_gyro()   # 执行标定

print("\n>>> 进入验证阶段（保持静止，观察校准后读数）")
print("扣除零偏后各轴应接近 0")
print("格式: gx_cal, gy_cal, gz_cal (LSB)")
print("---")

try:
    while True:
        gx_c, gy_c, gz_c = get_corrected_gyro(GYRO_BIAS)
        txt = "G_cal:{:6d},{:6d},{:6d}  (单位: LSB)"
        print(txt.format(gx_c, gy_c, gz_c))
        sleep(0.05)

except KeyboardInterrupt:
    print("\n实验结束")
    print("提示: 将 GYRO_BIAS 的值记下来，")
    print("      实验四、五中可以直接使用")
