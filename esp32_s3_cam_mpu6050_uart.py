from machine import I2C, Pin, UART
from time import sleep

'''
ESP32-S3-CAM:
  I2C: SCL=GPIO21, SDA=GPIO47 (接 MPU6050)
  UART: GPIO43(TX) -> V1 RX, GPIO44(RX) <- V1 TX (用 UART0, 默认引脚)
'''

MPU6050_ADDR = 0x68

# ---- I2C 初始化 ----
i2c = I2C(0, scl=Pin(21), sda=Pin(47), freq=400000)
i2c.writeto_mem(MPU6050_ADDR, 0x6B, b'\x00')
sleep(0.1)

uart = UART(1, baudrate=115200, tx=Pin(1), rx=Pin(2))  # 用 UART1+GPIO1/2


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


print("=" * 50)
print("ESP32-S3-CAM: MPU6050 -> UART 发送")
print("=" * 50)
print("UART1 | TX=GPIO1 -> V1 RX=GPIO23 | RX=GPIO2 <- V1 TX=GPIO22")
print("格式: ax,ay,az,gx,gy,gz (原始值)")
print("---")

count = 0
while True:
    ax, ay, az, gx, gy, gz = read_mpu6050()
    line = "{},{},{},{},{},{}\n".format(ax, ay, az, gx, gy, gz)
    uart.write(line)
    count += 1
    if count % 25 == 0:
        print("已发送 {} 组 | ax={}, ay={}, az={}".format(count, ax, ay, az))
    sleep(0.1)
