from machine import Pin, PWM, ADC
import time

# =====================================
# 电机PWM参数
# =====================================
PWM_FREQ = 20000

# =====================================
# 左电机
# =====================================
pin_m1_in1 = Pin(13, Pin.OUT)
pin_m1_in2 = Pin(15, Pin.OUT)

pwm_m1_in1 = PWM(pin_m1_in1, freq=PWM_FREQ, duty=0)
pwm_m1_in2 = PWM(pin_m1_in2, freq=PWM_FREQ, duty=0)

# =====================================
# 右电机
# =====================================
pin_m2_in1 = Pin(14, Pin.OUT)
pin_m2_in2 = Pin(25, Pin.OUT)

pwm_m2_in1 = PWM(pin_m2_in1, freq=PWM_FREQ, duty=0)
pwm_m2_in2 = PWM(pin_m2_in2, freq=PWM_FREQ, duty=0)

# =====================================
# 五路循迹传感器
# =====================================
adc1 = ADC(Pin(36))   # 最左（WiFi使用时改到GPIO36，避开ADC2冲突）
adc2 = ADC(Pin(33))   # 左中
adc3 = ADC(Pin(32))   # 中间
adc4 = ADC(Pin(35))   # 右中
adc5 = ADC(Pin(34))   # 最右

THRESHOLD = 1200

# =====================================
# 循迹参数
# =====================================
BASE_SPEED = 66
KP = 12
KD = 8

# 弯道动态限速
SPEED_STRAIGHT = 66    # 直道
SPEED_SLIGHT   = 53    # 小弯
SPEED_TURN     = 46    # 急弯

# 脱线恢复参数
RECOVER_SPEED = 40

# 直角弯专用参数
SHARP_TURN_SPEED_FWD = 59    # 外侧轮速度（向前）
SHARP_TURN_SPEED_BWD = 55    # 内侧轮速度（向后）
SHARP_TURN_DURATION = 80     # 持续旋转时间 (ms)

last_error = 0

# =====================================
#  电机控制
# =====================================
def set_motor1_speed(speed):
    speed = max(-100, min(100, speed))
    if speed > 0:
        pwm_m1_in1.duty(0)
        pwm_m1_in2.duty(int(speed * 1023 / 100))
    elif speed < 0:
        pwm_m1_in1.duty(int(abs(speed) * 1023 / 100))
        pwm_m1_in2.duty(0)
    else:
        pwm_m1_in1.duty(0)
        pwm_m1_in2.duty(0)


def set_motor2_speed(speed):
    speed = max(-100, min(100, speed))
    if speed > 0:
        pwm_m2_in1.duty(0)
        pwm_m2_in2.duty(int(speed * 1023 / 100))
    elif speed < 0:
        pwm_m2_in1.duty(int(abs(speed) * 1023 / 100))
        pwm_m2_in2.duty(0)
    else:
        pwm_m2_in1.duty(0)
        pwm_m2_in2.duty(0)


def read_track():
    s1 = 1 if adc1.read() > THRESHOLD else 0
    s2 = 1 if adc2.read() > THRESHOLD else 0
    s3 = 1 if adc3.read() > THRESHOLD else 0
    s4 = 1 if adc4.read() > THRESHOLD else 0
    s5 = 1 if adc5.read() > THRESHOLD else 0
    return [s1, s2, s3, s4, s5]


def calc_error(sensor):
    weights = [-4, -2, 0, 2, 4]
    error = 0
    for i in range(5):
        error += sensor[i] * weights[i]
    return error


def get_base_speed(error):
    abs_err = abs(error)
    if abs_err >= 4:
        return SPEED_TURN
    elif abs_err >= 2:
        return SPEED_SLIGHT
    else:
        return SPEED_STRAIGHT


def main():
    global last_error

    print("循迹启动")
    print("KP={}, KD={}, 弯道速度: {}/{}/{}".format(
        KP, KD, SPEED_TURN, SPEED_SLIGHT, SPEED_STRAIGHT))

    try:
        while True:
            sensor = read_track()

            # ============================================================
            #  1. 直角弯检测（在 PD 控制之前拦截）
            #     原理：当只有最外侧1~2个传感器检测到黑线时，
            #          说明线以接近90度拐弯，PD已经来不及跟
            # ============================================================
            # 左直角：最左侧1~2个传感器检测到黑线
            if sensor in [[1, 1, 0, 0, 0], [1, 0, 0, 0, 0]]:
                # 左轮后退，右轮前进 → 原地向左旋转
                set_motor1_speed(-SHARP_TURN_SPEED_BWD)
                set_motor2_speed(SHARP_TURN_SPEED_FWD)
                time.sleep_ms(SHARP_TURN_DURATION)
                last_error = -8   # 复位误差方向记录
                continue

            # 右直角：最右侧1~2个传感器检测到黑线
            if sensor in [[0, 0, 0, 1, 1], [0, 0, 0, 0, 1]]:
                # 左轮前进，右轮后退 → 原地向右旋转
                set_motor1_speed(SHARP_TURN_SPEED_FWD)
                set_motor2_speed(-SHARP_TURN_SPEED_BWD)
                time.sleep_ms(SHARP_TURN_DURATION)
                last_error = 8    # 复位误差方向记录
                continue

            # ============================================================
            #  2. 脱线恢复（全部白底，说明彻底失去了线）
            # ============================================================
            if sensor == [0, 0, 0, 0, 0]:
                if last_error < 0:
                    set_motor1_speed(RECOVER_SPEED)
                    set_motor2_speed(-RECOVER_SPEED)
                else:
                    set_motor1_speed(-RECOVER_SPEED)
                    set_motor2_speed(RECOVER_SPEED)

                time.sleep_ms(30)

                retry = 0
                while read_track() == [0, 0, 0, 0, 0]:
                    retry += 1
                    if retry < 10:
                        continue
                    elif retry < 20:
                        set_motor1_speed(-RECOVER_SPEED)
                        set_motor2_speed(-RECOVER_SPEED)
                        time.sleep_ms(50)
                    else:
                        set_motor1_speed(0)
                        set_motor2_speed(0)
                        time.sleep_ms(100)
                        break

                last_error = 0
                continue

            # ============================================================
            #  3. 正常 PD 循迹控制
            # ============================================================
            error = calc_error(sensor)
            derivative = error - last_error
            last_error = error

            turn = error * KP + derivative * KD
            current_base = get_base_speed(error)

            left_speed = current_base + turn
            right_speed = current_base - turn

            left_speed = max(-100, min(100, left_speed))
            right_speed = max(-100, min(100, right_speed))

            set_motor1_speed(left_speed)
            set_motor2_speed(right_speed)

            time.sleep_ms(10)

    except KeyboardInterrupt:
        print("停止")
        set_motor1_speed(0)
        set_motor2_speed(0)
        pwm_m1_in1.deinit()
        pwm_m1_in2.deinit()
        pwm_m2_in1.deinit()
        pwm_m2_in2.deinit()
        print("电机已停止")


if __name__ == "__main__":
    main()
