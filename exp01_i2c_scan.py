# exp01_i2c_scan.py
# 实验一：I²C 设备扫描
# 连接 MPU6050，运行 scan() 确认看到 0x68，
# 读取 WHO_AM_I 寄存器 (0x75) 验证通信正常
#
# 接线: VCC→3.3V, GND→GND,
#       SCL→GPIO21, SDA→GPIO47, AD0→GND
#
# 用法: 在 Thonny / Mu / ampy 中运行，
#       或逐行粘贴到 REPL 执行
# ============================================

from machine import I2C, Pin
from time import sleep

MPU6050_ADDR = 0x68   # AD0=GND 时的默认地址

print("=" * 40)
print("实验一：I²C 设备扫描 & 通信验证")
print("=" * 40)

# 1. 初始化 I²C (SCL=GPIO21, SDA=GPIO47, 400kHz)
i2c = I2C(0, scl=Pin(21), sda=Pin(47), freq=400000)

# 2. 扫描 I²C 总线
print("\n>>> 扫描 I²C 总线...")
devices = i2c.scan()
hex_addrs = [hex(d) for d in devices]
print("发现设备:", hex_addrs)

if MPU6050_ADDR in devices:
    print("  [✓] 发现 MPU6050! (地址 0x{:02X})".format(MPU6050_ADDR))
else:
    print("  [×] 未发现 MPU6050!")
    print("  请检查:")
    print("    1. VCC→3.3V, GND→GND 是否正确")
    print("    2. SCL→GPIO21, SDA→GPIO47")
    print("    3. AD0→GND (地址 0x68)")
    print("    4. 模块上的 LED 是否亮起")
    raise SystemExit("接线检查后重新运行")

# 3. 唤醒 MPU6050 (PWR_MGMT_1 寄存器 0x6B)
print("\n>>> 唤醒 MPU6050...")
i2c.writeto_mem(MPU6050_ADDR, 0x6B, b'\x00')
print("  PWR_MGMT_1 ← 0x00  (退出睡眠模式)  ✓")
sleep(0.1)

# 4. 读取 WHO_AM_I 寄存器 (0x75) 验证
print("\n>>> 验证 WHO_AM_I 寄存器 (0x75)...")
whoami = i2c.readfrom_mem(MPU6050_ADDR, 0x75, 1)[0]
print("  WHO_AM_I = 0x{:02X}".format(whoami))

if whoami == MPU6050_ADDR:
    print("  [✓] 验证通过! MPU6050 通信正常!")
else:
    print("  [!] 预期 0x{:02X}, 实际 0x{:02X}".format(MPU6050_ADDR, whoami))
    print("  请检查模块或接线")

print("\n" + "=" * 40)
print("实验一完成")
print("=" * 40)
print("\n系统信息:")
print("  I²C 控制器:", i2c)
print("  频率:", i2c, "Hz")
print("  SCL: GPIO21, SDA: GPIO47")
print("  地址 0x68 = MPU6050 (AD0=GND)")
print("  共发现", len(devices), "个设备")
