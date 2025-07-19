import RPi.GPIO as GPIO
import time
import atexit

# ==== Chân kết nối cảm biến ====
SENSOR_LEFT = 23
SENSOR_CENTER = 24
SENSOR_RIGHT = 25

# ==== Chân điều khiển motor L9110 ====
IN1 = 6   # Motor trái
IN2 = 5
ENA = 12

IN3 = 22  # Motor phải
IN4 = 27
ENB = 13

# ==== Thiết lập GPIO ====
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup([SENSOR_LEFT, SENSOR_CENTER, SENSOR_RIGHT], GPIO.IN)
GPIO.setup([IN1, IN2, ENA, IN3, IN4, ENB], GPIO.OUT)

pwm_ena = GPIO.PWM(ENA, 1000)
pwm_enb = GPIO.PWM(ENB, 1000)
pwm_ena.start(0)
pwm_enb.start(0)

# ==== Hàm điều khiển động cơ ====
def set_motor(l_in1, l_in2, r_in3, r_in4, speed_l, speed_r):
    GPIO.output(IN1, l_in1)
    GPIO.output(IN2, l_in2)
    GPIO.output(IN3, r_in3)
    GPIO.output(IN4, r_in4)
    pwm_ena.ChangeDutyCycle(max(0, min(speed_l, 100)))
    pwm_enb.ChangeDutyCycle(max(0, min(speed_r, 100)))

def stop():
    set_motor(0, 0, 0, 0, 0, 0)

def forward(speed=50):
    set_motor(0, 1, 1, 0, speed, speed)

def left_soft(speed=50):
    set_motor(0, 1, 1, 0, speed//2, speed)

def left_hard(speed=60):
    set_motor(1, 0, 1, 0, speed, speed)

def right_soft(speed=50):
    set_motor(0, 1, 1, 0, speed, speed//2)

def right_hard(speed=60):
    set_motor(0, 1, 0, 1, speed, speed)

def search_forward(speed=30):
    set_motor(0, 1, 1, 0, speed, speed)

# ==== Dọn dẹp khi kết thúc chương trình ====
def cleanup():
    stop()
    pwm_ena.stop()
    pwm_enb.stop()
    GPIO.cleanup()
    print("Đã dọn dẹp GPIO")

atexit.register(cleanup)

# ==== Main loop dò line ====
print("Bắt đầu dò line...")
try:
    while True:
        L = GPIO.input(SENSOR_LEFT)
        C = GPIO.input(SENSOR_CENTER)
        R = GPIO.input(SENSOR_RIGHT)

        print(f"Cảm biến: L={L}, C={C}, R={R}")

        if C == 1 and L == 0 and R == 0:
            forward(60)  # Đi thẳng nhanh
        elif C == 1 and L == 1 and R == 0:
            left_soft(50)
        elif C == 0 and L == 1 and R == 0:
            left_hard(60)
        elif C == 1 and L == 0 and R == 1:
            right_soft(50)
        elif C == 0 and L == 0 and R == 1:
            right_hard(60)
        elif C == 0 and L == 0 and R == 0:
            search_forward(30)  # Mất line, đi chậm để tìm
        else:
            forward(55)  # Mặc định đi thẳng

        time.sleep(0.03)

except KeyboardInterrupt:
    cleanup()
