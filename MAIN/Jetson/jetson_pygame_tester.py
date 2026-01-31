import pygame
import serial
import threading
import time
import collections
import matplotlib.pyplot as plt
import numpy as np

# ======================
# Serial config
# ======================
PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
ser = serial.Serial(PORT, BAUDRATE, timeout=0.1)

# ======================
# Control params
# ======================
SERVO_MIN = 0
SERVO_MAX = 180
SERVO_STEP = 2

MAX_RPM = 500.0
RPM_STEP = 5.0

servo_angle = 90
target_rpm = 0.0
current_rpm = 0.0

# ======================
# RPM history
# ======================
RPM_HISTORY_LEN = 300
rpm_history = collections.deque(maxlen=RPM_HISTORY_LEN)

# ======================
# Serial RX thread
# ======================
def read_serial():
    global current_rpm
    while True:
        try:
            line = ser.readline().decode().strip()
            if line:
                current_rpm = float(line)
                rpm_history.append(current_rpm)
        except:
            pass

threading.Thread(target=read_serial, daemon=True).start()

# ======================
# Pygame init
# ======================
pygame.init()
WIDTH, HEIGHT = 600, 400
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("WASD Slider Control")

font = pygame.font.SysFont(None, 28)
clock = pygame.time.Clock()

# ======================
# Matplotlib init
# ======================
plt.ion()
fig, ax = plt.subplots()
line, = ax.plot([], [], lw=2)
ax.set_ylim(-550, 550)
ax.set_xlim(0, RPM_HISTORY_LEN)
ax.set_ylabel("RPM")
ax.set_xlabel("Samples")
ax.set_title("Motor RPM (Real-time)")
ax.grid(True)

# ======================
# Helpers
# ======================
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def draw_slider(x, y, w, h, value_norm, label):
    pygame.draw.rect(screen, (80, 80, 80), (x, y, w, h), 2)
    if w > h:  # horizontal
        knob_x = x + int(value_norm * w)
        pygame.draw.circle(screen, (0, 200, 255), (knob_x, y + h // 2), 8)
    else:      # vertical
        knob_y = y + h - int(value_norm * h)
        pygame.draw.circle(screen, (255, 100, 100), (x + w // 2, knob_y), 8)

    txt = font.render(label, True, (255, 255, 255))
    screen.blit(txt, (x, y - 25))

# ======================
# Main loop
# ======================
running = True
while running:
    clock.tick(60)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_a:
                servo_angle -= SERVO_STEP
            elif event.key == pygame.K_d:
                servo_angle += SERVO_STEP
            elif event.key == pygame.K_w:
                target_rpm += RPM_STEP
            elif event.key == pygame.K_s:
                target_rpm -= RPM_STEP

    servo_angle = clamp(servo_angle, SERVO_MIN, SERVO_MAX)
    target_rpm = clamp(target_rpm, -MAX_RPM, MAX_RPM)

    # Send to ESP32
    cmd = f"{servo_angle} {target_rpm:.2f}\n"
    ser.write(cmd.encode())

    # ---------- UI ----------
    screen.fill((20, 20, 20))

    # Horizontal slider (servo)
    servo_norm = (servo_angle - SERVO_MIN) / (SERVO_MAX - SERVO_MIN)
    draw_slider(100, 300, 400, 20, servo_norm, f"Servo Angle: {servo_angle} deg")

    # Vertical slider (rpm)
    rpm_norm = (target_rpm + MAX_RPM) / (2 * MAX_RPM)
    draw_slider(50, 50, 20, 250, rpm_norm, f"Target RPM: {target_rpm:.1f}")

    # Text
    txt = font.render(f"Current RPM: {current_rpm:.1f}", True, (0, 255, 0))
    screen.blit(txt, (100, 50))

    pygame.display.flip()

    # ---------- Plot ----------
    if len(rpm_history) > 5:
        y = np.array(rpm_history)
        x = np.arange(len(y))
        line.set_data(x, y)
        ax.set_xlim(0, RPM_HISTORY_LEN)
        fig.canvas.draw()
        fig.canvas.flush_events()

pygame.quit()
ser.close()
