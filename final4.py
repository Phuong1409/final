import RPi.GPIO as GPIO
import time
import atexit

# ========== CẤU HÌNH ==========

# Chân cảm biến line (L - C - R)
SENSOR_LEFT = 23
SENSOR_CENTER = 24
SENSOR_RIGHT = 25

# Chân điều khiển động cơ
MOTOR_PINS = {
    'IN1': 6,   # Trái
    'IN2': 5,
    'ENA': 12,
    'IN3': 22,  # Phải
    'IN4': 27,
    'ENB': 13
}

# Tốc độ cơ bản
base_speed = 30  # <== chỉnh thông số duy nhất để thay đổi toàn bộ tốc độ

# PID thông số
pid = None  # sẽ khởi tạo sau

# ========== KHỞI TẠO GPIO ==========

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup([SENSOR_LEFT, SENSOR_CENTER, SENSOR_RIGHT], GPIO.IN)
GPIO.setup(list(MOTOR_PINS.values()), GPIO.OUT)

# PWM
pwm_ena = GPIO.PWM(MOTOR_PINS['ENA'], 500)
pwm_enb = GPIO.PWM(MOTOR_PINS['ENB'], 500)
pwm_ena.start(0)
pwm_enb.start(0)

# ========== TIỆN ÍCH ==========

def clamp(value, min_val=0, max_val=100):
    return max(min_val, min(value, max_val))

# ========== ĐIỀU KHIỂN ĐỘNG CƠ ==========

def set_motor(l_in1, l_in2, r_in3, r_in4, speed_l, speed_r):
    GPIO.output(MOTOR_PINS['IN1'], l_in1)
    GPIO.output(MOTOR_PINS['IN2'], l_in2)
    GPIO.output(MOTOR_PINS['IN3'], r_in3)
    GPIO.output(MOTOR_PINS['IN4'], r_in4)
    pwm_ena.ChangeDutyCycle(clamp(speed_l))
    pwm_enb.ChangeDutyCycle(clamp(speed_r))

def stop():
    set_motor(0, 0, 0, 0, 0, 0)

def forward(speed=base_speed):
    set_motor(0, 1, 1, 0, speed, speed)

def rotate_left(speed=base_speed):
    set_motor(1, 0, 0, 1, speed, speed)

def rotate_right(speed=base_speed):
    set_motor(0, 1, 1, 0, speed, speed)

# ========== PID CLASS ==========

class PID:
    def __init__(self, kp=1.0, ki=0.02, kd=0.15):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0
        self.prev_time = time.time()

    def compute(self, error):
        now = time.time()
        dt = now - self.prev_time if now > self.prev_time else 0.01
        self.prev_time = now

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return output

# ========== ĐỌC CẢM BIẾN ==========

def read_sensors():
    return GPIO.input(SENSOR_LEFT), GPIO.input(SENSOR_CENTER), GPIO.input(SENSOR_RIGHT)

# ========== DÒ LINE ==========

last_seen = 1  # 1: trái, -1: phải

def line_following():
    global last_seen
    L, C, R = read_sensors()
    print(f"Cảm biến: L={L}, C={C}, R={R}")

    error = 0
    if L == 0 and C == 1 and R == 0:
        error = 0
    elif L == 1 and C == 1 and R == 0:
        error = 0.5
        last_seen = 1
    elif L == 1 and C == 0 and R == 0:
        error = 1.0
        last_seen = 1
    elif L == 1 and C == 0 and R == 1:
        error = 0  # ngã tư
    elif L == 0 and C == 1 and R == 1:
        error = -0.5
        last_seen = -1
    elif L == 0 and C == 0 and R == 1:
        error = -1.0
        last_seen = -1
    elif L == 0 and C == 0 and R == 0:
        print("⚠️ Mất line, đang tìm lại...")
        search_for_line()
        return

    # Tính hiệu chỉnh tốc độ
    correction = pid.compute(error)
    left_speed = clamp(base_speed - correction)
    right_speed = clamp(base_speed + correction)
    set_motor(0, 1, 1, 0, left_speed, right_speed)

# ========== TÌM LINE KHI MẤT ==========

def search_for_line():
    global last_seen
    if last_seen == 1:
        rotate_left(base_speed)
    else:
        rotate_right(base_speed)

# ========== DỌN DẸP ==========

def cleanup():
    stop()
    pwm_ena.stop()
    pwm_enb.stop()
    GPIO.cleanup()
    print("Đã thoát và dọn dẹp GPIO")

atexit.register(cleanup)

# ========== MAIN ==========

pid = PID(kp=1.4, ki=0.02, kd=0.25)

print("🚗 Khởi động robot dò line...")
try:
    while True:
        line_following()
        time.sleep(0.01)
except KeyboardInterrupt:
    cleanup()
