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
    pwm_ena.ChangeDutyCycle(speed_l)
    pwm_enb.ChangeDutyCycle(speed_r)

def stop():
    set_motor(0, 0, 0, 0, 0, 0)

def forward(speed=3):
    set_motor(0, 1, 1, 0, speed, speed)

def left(speed=1):
    set_motor(1, 0, 1, 0, speed, speed)

def right(speed=1):
    set_motor(0, 1, 0, 1, speed, speed)

# === PID điều khiển ===
class PID:
    def __init__(self, kp=22, ki=0, kd=8):
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

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

# === Đọc cảm biến ===
def read_sensors():
    L = GPIO.input(SENSOR_LEFT)
    C = GPIO.input(SENSOR_CENTER)
    R = GPIO.input(SENSOR_RIGHT)
    return (L, C, R)

# === Dò line chính ===
def line_following():
    sensors = read_sensors()
    L, C, R = sensors
    print(f"Cảm biến: L={L}, C={C}, R={R}")

    # Quy đổi trạng thái cảm biến sang sai số
    if C == 1 and L == 0 and R == 0:
        error = 0  # đi thẳng
    elif L == 1 and C == 0:
        error = 1  # lệch trái
    elif R == 1 and C == 0:
        error = -1  # lệch phải
    elif L == 1 and R == 1:
        error = 0  # có thể là ngã tư – coi như đi thẳng
    else:
        error = 0  # mặc định

    # Điều khiển PID
    correction = pid.compute(error)
    base_speed = 3

    left_speed = max(1, min(base_speed - correction, 100))
    right_speed = max(1, min(base_speed + correction, 100))
    set_motor(0, 1, 1, 0, left_speed, right_speed)

# === Dọn dẹp khi kết thúc ===
def cleanup():
    stop()
    pwm_ena.stop()
    pwm_enb.stop()
    GPIO.cleanup()
    print("Đã thoát và dọn dẹp GPIO")

atexit.register(cleanup)

# === Chạy chương trình ===
pid = PID(kp=22, ki=0, kd=8)
print("Khởi động dò line...")
try:
    while True:
        line_following()
        time.sleep(0.01)
except KeyboardInterrupt:
    cleanup()
