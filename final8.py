import RPi.GPIO as GPIO
import time
import atexit

# === Chân GPIO ===
SENSOR_LEFT = 23
SENSOR_CENTER = 24
SENSOR_RIGHT = 25

MOTOR_PINS = {
    'IN1': 6, 'IN2': 5, 'ENA': 12,  # Motor trái
    'IN3': 22, 'IN4': 27, 'ENB': 13  # Motor phải
}

# === Cài đặt GPIO ===
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup([SENSOR_LEFT, SENSOR_CENTER, SENSOR_RIGHT], GPIO.IN)
GPIO.setup(list(MOTOR_PINS.values()), GPIO.OUT)

pwm_ena = GPIO.PWM(MOTOR_PINS['ENA'], 1000)
pwm_enb = GPIO.PWM(MOTOR_PINS['ENB'], 1000)
pwm_ena.start(0)
pwm_enb.start(0)

# === Hàm điều khiển động cơ ===
def set_motor(l_in1, l_in2, r_in3, r_in4, speed_l, speed_r):
    GPIO.output(MOTOR_PINS['IN1'], l_in1)
    GPIO.output(MOTOR_PINS['IN2'], l_in2)
    GPIO.output(MOTOR_PINS['IN3'], r_in3)
    GPIO.output(MOTOR_PINS['IN4'], r_in4)
    pwm_ena.ChangeDutyCycle(max(0, min(speed_l, 100)))
    pwm_enb.ChangeDutyCycle(max(0, min(speed_r, 100)))

def stop():
    set_motor(0, 0, 0, 0, 0, 0)

def forward(speed_l=12, speed_r=12):
    set_motor(0, 1, 1, 0, speed_l, speed_r)

# === PID Controller ===
class PID:
    def __init__(self, kp=25, ki=0, kd=2):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()

    def compute(self, error):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

# === Đọc cảm biến ===
def read_sensors():
    L = GPIO.input(SENSOR_LEFT)
    C = GPIO.input(SENSOR_CENTER)
    R = GPIO.input(SENSOR_RIGHT)
    return L, C, R

# === Xử lý điều hướng bằng PID ===
def line_following():
    L, C, R = read_sensors()
    print(f"L={L} C={C} R={R}")

    # Xác định lỗi (error) theo hướng lệch
    if L == 1 and C == 0 and R == 0:
        error = 2
    elif L == 1 and C == 1 and R == 0:
        error = 1
    elif L == 0 and C == 1 and R == 0:
        error = 0
    elif L == 0 and C == 1 and R == 1:
        error = -1
    elif L == 0 and C == 0 and R == 1:
        error = -2
    elif L == 0 and C == 0 and R == 0:
        error = 0  # Mất line, vẫn tiến tới
    else:
        error = 0  # Các trường hợp còn lại coi như đi thẳng

    # Tính hiệu chỉnh bằng PID
    correction = pid.compute(error)

    base_speed = 12
    left_speed = base_speed - correction
    right_speed = base_speed + correction

    # Giới hạn trong khoảng 0–100
    left_speed = max(0, min(100, left_speed))
    right_speed = max(0, min(100, right_speed))

    forward(speed_l=left_speed, speed_r=right_speed)

# === Dọn dẹp GPIO khi thoát ===
def cleanup():
    stop()
    pwm_ena.stop()
    pwm_enb.stop()
    GPIO.cleanup()
    print("Dọn dẹp GPIO và thoát")

atexit.register(cleanup)

# === Bắt đầu chạy ===
pid = PID(kp=25, ki=0, kd=2)
print("Bắt đầu dò line với PID...")

try:
    while True:
        line_following()
        time.sleep(0.005)  # Chu kỳ phản hồi nhanh
except KeyboardInterrupt:
    cleanup()
