# exp05_car_data_logging.py
# 实验五：姿态数据记录与分析
# 将 MPU6050 固定到小车上，记录前进/后退/转弯时的
# 完整 6 轴数据，分析姿态变化规律
#
# 应用场景 (PPT 第12页):
#   a) 转向检测 (Z轴角速度)
#   b) 加速度/碰撞检测 (前进方向 ax)
#   c) 倾斜/坡道检测 (az/重力分量)
#
# 操作流程:
#   1. 水平安装 MPU6050 到小车中央
#   2. 运行程序，观察实时状态判断
#   3. 依次执行: 静止->前进->静止->后退->左转->右转->急停
#   4. 分析各阶段数据规律
# ============================================

from machine import I2C, Pin
from time import sleep, ticks_ms, ticks_diff
import math

MPU6050_ADDR = 0x68
ACCEL_SCALE = 16384.0
GYRO_SCALE  = 131.0
BUFFER_SIZE = 200    # 数据缓冲区大小 (约 20s @ 10Hz)

i2c = I2C(0, scl=Pin(21), sda=Pin(47), freq=400000)
i2c.writeto_mem(MPU6050_ADDR, 0x6B, b'\x00')
sleep(0.1)

# 实验三的零偏值 (替换为实际标定结果)
GYRO_BIAS = (0, 0, 0)


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


def detect_motion(ax, ay, az, gx, gy, gz):
    """判断小车运动状态 (PPT 第12页)
       返回: 0=静止, 1=前进, 2=后退, 3=左转, 4=右转"""
    acc_mag = math.sqrt(ax*ax + ay*ay + az*az)

    # 静止: 总加速度接近 1g 且角速度很小
    if abs(acc_mag - ACCEL_SCALE) < ACCEL_SCALE * 0.1:
        gyro_mag = math.sqrt(gx*gx + gy*gy + gz*gz)
        if gyro_mag < GYRO_SCALE * 10:   # < 10 deg/s
            return 0

    # 左转/右转: 根据 Z 轴角速度 (gz 正值=左转)
    if abs(gz) > GYRO_SCALE * 15:        # > 15 deg/s
        return 3 if gz > 0 else 4

    # 前进/后退: 根据 X 轴加速度 (ax 正值=前进加速)
    if abs(ax) > ACCEL_SCALE * 0.2:      # > 0.2g
        return 1 if ax > 0 else 2

    return 0


def state_name(s):
    names = ["静止", "前进", "后退", "左转", "右转"]
    return names[s] if 0 <= s <= 4 else "未知"


# ---- 数据缓冲区 ----
buffer = []
print("=" * 50)
print("实验五：姿态数据记录与分析")
print("=" * 50)
print("操作流程:")
print("  静止 5s -> 前进 5s -> 静止 3s -> 后退 5s")
print("  左转 3s -> 右转 3s -> 急停 -> 静止")
print("---")
print("数据格式 (CSV):")
print("  time(ms), ax(g), ay(g), az(g), gx(dps), gy(dps), gz(dps), 状态")
print("---")
print("time,ax,ay,az,gx,gy,gz,state")
print("---")

try:
    while len(buffer) < BUFFER_SIZE:
        ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw = read_mpu6050()

        ax = ax_raw / ACCEL_SCALE
        ay = ay_raw / ACCEL_SCALE
        az = az_raw / ACCEL_SCALE
        gx = (gx_raw - GYRO_BIAS[0]) / GYRO_SCALE
        gy = (gy_raw - GYRO_BIAS[1]) / GYRO_SCALE
        gz = (gz_raw - GYRO_BIAS[2]) / GYRO_SCALE

        state = detect_motion(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw)
        buffer.append((ticks_ms(), ax, ay, az, gx, gy, gz, state))

        # CSV 格式实时输出
        t_now = ticks_ms()
        csv = "{},{:.3f},{:.3f},{:.3f},{:.1f},{:.1f},{:.1f},{}"
        print(csv.format(t_now, ax, ay, az, gx, gy, gz, state_name(state)))

        sleep(0.1)

    # ---- 缓冲区满，数据分析 ----
    print("\n=== 缓冲区已满 ({}) 组，分析报告 ===".format(BUFFER_SIZE))

    # 各运动状态统计
    state_counts = [0, 0, 0, 0, 0]
    for _, _, _, _, _, _, _, s in buffer:
        if 0 <= s <= 4:
            state_counts[s] += 1

    print("\n运动状态占比:")
    for s in range(5):
        pct = state_counts[s] * 100.0 / BUFFER_SIZE
        print("  {}: {} 组 ({:.1f}%)".format(state_name(s), state_counts[s], pct))

    # 各轴数据范围
    values = list(zip(*buffer))
    ax_vals = [v for v in values[1]]
    ay_vals = [v for v in values[2]]
    az_vals = [v for v in values[3]]
    gz_vals = [v for v in values[6]]

    print("\n数据范围分析:")
    print("  ax: {:.3f} ~ {:.3f} g".format(min(ax_vals), max(ax_vals)))
    print("  ay: {:.3f} ~ {:.3f} g".format(min(ay_vals), max(ay_vals)))
    print("  az: {:.3f} ~ {:.3f} g".format(min(az_vals), max(az_vals)))
    print("  gz: {:.1f} ~ {:.1f} deg/s".format(min(gz_vals), max(gz_vals)))

    print("\n观察与分析要点 (PPT 第12页):")
    print("  1. 前进/后退时 ax 有明显正/负跳变")
    print("  2. 转弯时 gz 出现较大正值(左转)/负值(右转)")
    print("  3. 急停时加速度出现反向尖峰 (碰撞检测!)")
    print("  4. 静止时 az 稳定在 1.0g 附近")
    print("  5. 上坡时 az 下降, ax 增加 (倾斜检测)")
    print("  6. 可将 CSV 数据粘贴到 Excel 画图观察更清楚")

    print("\n实验结束")

except KeyboardInterrupt:
    print("\n用户中断，已记录 {} 组数据".format(len(buffer)))
