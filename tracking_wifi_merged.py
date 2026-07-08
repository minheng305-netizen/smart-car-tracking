from machine import Pin, PWM, ADC, UART
import time
import network
import socket

'''
融合版：PD循迹 + WiFi/UDP + MPU6050 UART接收
'''

'''================== 配置区 ================================================================'''
SSID = "Redmi K70 Pro"
PASSWORD = "12345678"
Server_IP = '10.140.33.70'
Server_Port = 8080

'''================== 电机PWM ================================================================'''
PWM_FREQ = 20000
pin_m1_in1 = Pin(13, Pin.OUT); pin_m1_in2 = Pin(15, Pin.OUT)
pwm_m1_in1 = PWM(pin_m1_in1, freq=PWM_FREQ, duty=0)
pwm_m1_in2 = PWM(pin_m1_in2, freq=PWM_FREQ, duty=0)
pin_m2_in1 = Pin(14, Pin.OUT); pin_m2_in2 = Pin(25, Pin.OUT)
pwm_m2_in1 = PWM(pin_m2_in1, freq=PWM_FREQ, duty=0)
pwm_m2_in2 = PWM(pin_m2_in2, freq=PWM_FREQ, duty=0)

'''================== LED ==================================================================='''
led = Pin(22, Pin.OUT); led.value(0)

'''================== UART MPU6050 ========================================================='''
uart = UART(1, baudrate=115200, tx=Pin(22), rx=Pin(23))
mpu6050_count = 0
print("UART: RX=GPIO23, TX=GPIO22, 115200bps")

'''================== 5路循迹ADC ============================================================'''
adc1 = ADC(Pin(36)); adc2 = ADC(Pin(33)); adc3 = ADC(Pin(32))
adc4 = ADC(Pin(35)); adc5 = ADC(Pin(34))
THRESHOLD = 1200

'''================== PD参数 ================================================================'''
KP = 12; KD = 8
SPEED_STRAIGHT = 66; SPEED_SLIGHT = 53; SPEED_TURN = 46
RECOVER_SPEED = 40
SHARP_TURN_SPEED_FWD = 59; SHARP_TURN_SPEED_BWD = 55; SHARP_TURN_DURATION = 80
last_error = 0

'''================== 上报频率 ==============================================================='''
report_freq = 3; report_counter = 0
freq_intervals = [0, 100, 50, 25, 10]
freq_names = ['STOP', '0.5Hz', '1Hz', '2Hz', '5Hz']

'''================== 电机控制 ==============================================================='''
def set_motor1_speed(speed):
    speed = max(-100, min(100, speed))
    if speed > 0: pwm_m1_in1.duty(0); pwm_m1_in2.duty(int(speed*1023/100))
    elif speed < 0: pwm_m1_in1.duty(int(abs(speed)*1023/100)); pwm_m1_in2.duty(0)
    else: pwm_m1_in1.duty(0); pwm_m1_in2.duty(0)

def set_motor2_speed(speed):
    speed = max(-100, min(100, speed))
    if speed > 0: pwm_m2_in1.duty(0); pwm_m2_in2.duty(int(speed*1023/100))
    elif speed < 0: pwm_m2_in1.duty(int(abs(speed)*1023/100)); pwm_m2_in2.duty(0)
    else: pwm_m2_in1.duty(0); pwm_m2_in2.duty(0)

def motor(ls, rs): set_motor1_speed(ls); set_motor2_speed(rs)

'''================== WiFi/UDP =============================================================='''
def WIFI_connect():
    global wlan
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('connecting...')
        wlan.connect(SSID, PASSWORD)
        while not wlan.isconnected(): pass
    print('OK', wlan.ifconfig())

def WIFI_start_udp():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    a = socket.getaddrinfo(Server_IP, Server_Port)[0][-1]
    s.connect(a); s.setblocking(False)
    s.sendto(b'PD Tracking started\r\n', a)
    return s

WIFI_connect(); My_socket = WIFI_start_udp(); time.sleep_ms(2000)

'''================== 循迹函数 =============================================================='''
def read_track():
    return [1 if adc1.read()>THRESHOLD else 0, 1 if adc2.read()>THRESHOLD else 0,
            1 if adc3.read()>THRESHOLD else 0, 1 if adc4.read()>THRESHOLD else 0,
            1 if adc5.read()>THRESHOLD else 0]

def calc_error(s):
    return s[0]*(-4)+s[1]*(-2)+s[2]*0+s[3]*2+s[4]*4

def get_base_speed(e):
    a=abs(e); return SPEED_TURN if a>=4 else (SPEED_SLIGHT if a>=2 else SPEED_STRAIGHT)

def read_adc():
    pins=[adc1,adc2,adc3,adc4,adc5]; d=[0]*5; v=[0]*5; st=[0]*5
    for i in range(5):
        for _ in range(5): d[i]+=pins[i].read()
        d[i]/=5; v[i]=d[i]*3.3/1024; st[i]=1 if v[i]>=1.4 else 0
    return v, st

'''================== 主循环 ================================================================'''
def main():
    global last_error, report_counter, mpu6050_count, report_freq
    print("PD KP={}, KD={}".format(KP, KD))
    loop=0; loop1=0; L_PWM=0; R_PWM=0
    try:
        while True:
            time.sleep_ms(10)
            loop=(loop+1)%25; loop1=(loop1+1)%5
            sensor=read_track()

            # 直角弯
            if sensor in [[1,1,0,0,0],[1,0,0,0,0]]:
                set_motor1_speed(-SHARP_TURN_SPEED_BWD); set_motor2_speed(SHARP_TURN_SPEED_FWD)
                time.sleep_ms(SHARP_TURN_DURATION); last_error=-8; continue
            if sensor in [[0,0,0,1,1],[0,0,0,0,1]]:
                set_motor1_speed(SHARP_TURN_SPEED_FWD); set_motor2_speed(-SHARP_TURN_SPEED_BWD)
                time.sleep_ms(SHARP_TURN_DURATION); last_error=8; continue

            # 脱线恢复
            if sensor==[0,0,0,0,0]:
                if last_error<0: set_motor1_speed(RECOVER_SPEED); set_motor2_speed(-RECOVER_SPEED)
                else: set_motor1_speed(-RECOVER_SPEED); set_motor2_speed(RECOVER_SPEED)
                time.sleep_ms(30)
                for _ in range(20):
                    if read_track()!=[0,0,0,0,0]: break
                    if _==10: set_motor1_speed(-RECOVER_SPEED); set_motor2_speed(-RECOVER_SPEED); time.sleep_ms(50)
                else: set_motor1_speed(0); set_motor2_speed(0); time.sleep_ms(100)
                last_error=0; continue

            # PD控制
            error=calc_error(sensor); d=error-last_error; last_error=error
            turn=error*KP+d*KD; base=get_base_speed(error)
            ls=max(-100,min(100,base+turn)); rs=max(-100,min(100,base-turn))
            set_motor1_speed(ls); set_motor2_speed(rs)

            # UART读MPU6050并转发
            if uart.any():
                try:
                    b=uart.readline()
                    if b:
                        mpu6050_count+=1
                        if mpu6050_count%5==0:
                            My_socket.sendto(b'MPU6050:'+b, (Server_IP, Server_Port))
                except: pass

            # UDP上报循迹
            if report_freq>0:
                report_counter+=1
                if report_counter>=freq_intervals[report_freq]:
                    report_counter=0; v,st=read_adc()
                    try:
                        My_socket.sendto('AO1-AO5: {:.2f},{:.2f},{:.2f},{:.2f},{:.2f}\r\n'.format(*v), (Server_IP, Server_Port))
                        My_socket.sendto('S0-S4: {}-{}-{}-{}-{}\n'.format(*st), (Server_IP, Server_Port))
                    except: pass

            # WiFi命令解析
            if loop1==0:
                try:
                    data, addr = My_socket.recvfrom(1024)
                    s=data.decode('utf-8').strip()
                    print('cmd:', s)
                    if s=='LED:1': led.value(0); My_socket.sendto(b'LED:1 open\r\n', (Server_IP, Server_Port))
                    elif s=='LED:0': led.value(1); My_socket.sendto(b'LED:0 closed\r\n', (Server_IP, Server_Port))
                    if s.startswith('FREQ:') and len(s)==6:
                        i=int(s[5])
                        if 0<=i<=4: report_freq=i; report_counter=0; My_socket.sendto('FREQ OK: {}\r\n'.format(freq_names[i]), (Server_IP, Server_Port))
                    if s[0]=='L' and s[1]==':':
                        L_PWM=(data[3]-48)*1000+(data[4]-48)*100+(data[5]-48)*10+(data[6]-48)
                        if L_PWM>=1023: L_PWM=1023
                        if s[2]=='-': L_PWM=-L_PWM
                    if len(s)>8 and s[8]=='R' and s[9]==':':
                        R_PWM=(data[11]-48)*1000+(data[12]-48)*100+(data[13]-48)*10+(data[14]-48)
                        if R_PWM>=1023: R_PWM=1023
                        if s[10]=='-': R_PWM=-R_PWM
                    if 'S' in s: L_PWM=0; R_PWM=0
                    motor(L_PWM, R_PWM)
                except OSError as e:
                    if e.args[0]!=11: raise

    except KeyboardInterrupt:
        set_motor1_speed(0); set_motor2_speed(0)
        pwm_m1_in1.deinit(); pwm_m1_in2.deinit()
        pwm_m2_in1.deinit(); pwm_m2_in2.deinit()
        print('Stopped')

if __name__=='__main__': main()

