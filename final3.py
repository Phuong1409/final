import RPi.GPIO as GPIO
import time
import atexit

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

# Thiết lập GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup([SENSOR_LEFT, SENSOR_CENTER, SENSOR_RIGHT], GPIO.IN)
GPIO.setup(list(MOTOR_PINS.values()), GPIO.OUT)

# PWM động cơ
pwm_ena = GPIO.PWM(MOTOR_PINS['ENA'], 500)
pwm_enb = GPIO.PWM(MOTOR_PINS['ENB'], 500)
pwm_ena.start(0)
pwm_enb.start(0)

# === Điều khiển động cơ ===
def set_motor(l_in1, l_in2, r_in3, r_in4, speed_l, speed_r):
    GPIO.output(MOTOR_PINS['IN1'], l_in1)
    GPIO.output(MOTOR_PINS['IN2'], l_in2)
    GPIO.output(MOTOR_PINS['IN3'], r_in3)
    GPIO.output(MOTOR_PINS['IN4'], r_in4)
    pwm_ena.ChangeDutyCycle(max(0, min(speed_l, 100)))
    pwm_enb.ChangeDutyCycle(max(0, min(speed_r, 100)))

def stop():
    set_motor(0, 0, 0, 0, 0, 0)

def forward(speed=20):
    set_motor(0, 1, 1, 0, speed, speed)

def left(speed=20):
    set_motor(1, 0, 1, 0, speed, speed)

def right(speed=20):
    set_motor(0, 1, 0, 1, speed, speed)

# === PID điều khiển ===
class PID:
    def __init__(self, kp=1.2, ki=0.05, kd=0.2):
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
        derivative = (error - self.prev_error) / dt if dt > 0 else 0

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

# === Đọc cảm biến ===
def read_sensors():
    return (GPIO.input(SENSOR_LEFT), GPIO.input(SENSOR_CENTER), GPIO.input(SENSOR_RIGHT))

# === Tìm lại line nếu bị lệch ===
def search_for_line(direction=1):
    # direction = 1: quay trái, -1: quay phải
    search_speed = 25
    if direction == 1:
        set_motor(1, 0, 0, 1, search_speed, search_speed)  # quay trái
    else:
        set_motor(0, 1, 1, 0, search_speed, search_speed)  # quay phải

# === Dò line chính ===
def line_following():
    L, C, R = read_sensors()
    print(f"Cảm biến: L={L}, C={C}, R={R}")

    if C == 1 and L == 0 and R == 0:
        error = 0  # đi thẳng
    elif L == 1 and C == 0:
        error = 1  # lệch trái
    elif R == 1 and C == 0:
        error = -1  # lệch phải
    elif L == 1 and R == 1:
        error = 0  # giao nhau, coi như đi thẳng
    elif C == 1 and (L == 1 or R == 1):
        error = 0  # nằm trên line rộng
    else:
        # Không thấy line, tìm lại
        print("Mất line, đang tìm lại...")
        search_for_line(direction=1)
        return

    # PID điều khiển
    correction = pid.compute(error)
    base_speed = 20

    left_speed = base_speed - correction
    right_speed = base_speed + correction

    set_motor(0, 1, 1, 0, left_speed, right_speed)

# === Dọn dẹp khi kết thúc ===
def cleanup():
    stop()
    pwm_ena.stop()
    pwm_enb.stop()
    GPIO.cleanup()
    print("Đã thoát và dọn dẹp GPIO")

atexit.register(cleanup)

# === Khởi động chương trình ===
pid = PID(kp=1.2, ki=0.05, kd=0.2)
print("Khởi động dò line...")
try:
    while True:
        line_following()
        time.sleep(0.01)
except KeyboardInterrupt:
    cleanup()
