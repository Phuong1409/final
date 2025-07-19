import RPi.GPIO as GPIO
import time
import atexit

# Cảm biến line
SENSOR_LEFT = 23
SENSOR_CENTER = 24
SENSOR_RIGHT = 25

# Động cơ
MOTOR_PINS = {
    'IN1': 6, 'IN2': 5, 'ENA': 12,
    'IN3': 22, 'IN4': 27, 'ENB': 13
}

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup([SENSOR_LEFT, SENSOR_CENTER, SENSOR_RIGHT], GPIO.IN)
GPIO.setup(list(MOTOR_PINS.values()), GPIO.OUT)

# PWM setup
pwm_ena = GPIO.PWM(MOTOR_PINS['ENA'], 1000)
pwm_enb = GPIO.PWM(MOTOR_PINS['ENB'], 1000)
pwm_ena.start(0)
pwm_enb.start(0)

# Hàm điều khiển động cơ
def set_motor(l_in1, l_in2, r_in3, r_in4, speed_l, speed_r):
    GPIO.output(MOTOR_PINS['IN1'], l_in1)
    GPIO.output(MOTOR_PINS['IN2'], l_in2)
    GPIO.output(MOTOR_PINS['IN3'], r_in3)
    GPIO.output(MOTOR_PINS['IN4'], r_in4)
    pwm_ena.ChangeDutyCycle(max(0, min(speed_l, 100)))
    pwm_enb.ChangeDutyCycle(max(0, min(speed_r, 100)))

def stop():
    set_motor(0, 0, 0, 0, 0, 0)

# === Điều khiển ===
def forward():
    set_motor(0, 1, 1, 0, 12, 12)  # cả hai bánh 12%

def turn_left():
    set_motor(0, 1, 1, 0, 2, 45)   # bánh trái chậm, bánh phải nhanh

def turn_right():
    set_motor(0, 1, 1, 0, 45, 2)   # bánh trái nhanh, bánh phải chậm

# Cảm biến
def read_sensors():
    return GPIO.input(SENSOR_LEFT), GPIO.input(SENSOR_CENTER), GPIO.input(SENSOR_RIGHT)

# Dò line
def line_following():
    L, C, R = read_sensors()
    print(f"L={L}, C={C}, R={R}")

    if C == 1 and L == 0 and R == 0:
        forward()
    elif L == 1 and C == 1:
        turn_left()
    elif L == 1 and C == 0 and R == 0:
        turn_left()
    elif R == 1 and C == 1:
        turn_right()
    elif R == 1 and C == 0 and L == 0:
        turn_right()
    elif L == 1 and R == 1:
        forward()
    elif L == 0 and C == 0 and R == 0:
        forward()  # hoặc tìm line
    else:
        forward()

# Dọn dẹp
def cleanup():
    stop()
    pwm_ena.stop()
    pwm_enb.stop()
    GPIO.cleanup()
    print("Đã dọn dẹp")

atexit.register(cleanup)

# Chạy
print("Robot đang dò line...")
try:
    while True:
        line_following()
        time.sleep(0.005)  # phản ứng nhanh hơn
except KeyboardInterrupt:
    cleanup()
