import pygame
import serial
import math
import threading

# ======================
# Serial config
# ======================
PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
ser = serial.Serial(PORT, BAUDRATE, timeout=0.1)

# ======================
# Control limits
# ======================
SERVO_MIN = 0
SERVO_MAX = 180
MAX_RPM = 500.0

# ======================
# Pygame init
# ======================
pygame.init()
WIDTH, HEIGHT = 500, 500
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Jetson Joystick Control")

CENTER = (WIDTH // 2, HEIGHT // 2)
RADIUS = 150
FONT = pygame.font.SysFont(None, 28)

clock = pygame.time.Clock()

# ======================
# State
# ======================
joy_x = 0.0  # [-1, 1]
joy_y = 0.0  # [-1, 1]
current_rpm = 0.0

# ======================
# Serial receive thread
# ======================
def read_serial():
    global current_rpm
    while True:
        try:
            line = ser.readline().decode().strip()
            if line:
                current_rpm = float(line)
        except:
            pass

threading.Thread(target=read_serial, daemon=True).start()

# ======================
# Helpers
# ======================
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def joystick_from_mouse(mx, my):
    dx = mx - CENTER[0]
    dy = CENTER[1] - my  # y axis inverted
    dist = math.hypot(dx, dy)
    if dist > RADIUS:
        dx *= RADIUS / dist
        dy *= RADIUS / dist
    return dx / RADIUS, dy / RADIUS

def map_control():
    servo = int((joy_x + 1) / 2 * (SERVO_MAX - SERVO_MIN) + SERVO_MIN)
    rpm = joy_y * MAX_RPM
    return servo, rpm

# ======================
# Main loop
# ======================
running = True
while running:
    clock.tick(60)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEMOTION:
            joy_x, joy_y = joystick_from_mouse(*event.pos)

    servo, rpm = map_control()
    cmd = f"{servo} {rpm:.2f}\n"
    ser.write(cmd.encode())

    screen.fill((20, 20, 20))

    # Draw joystick base
    pygame.draw.circle(screen, (80, 80, 80), CENTER, RADIUS, 3)

    # Draw knob
    knob_x = CENTER[0] + int(joy_x * RADIUS)
    knob_y = CENTER[1] - int(joy_y * RADIUS)
    pygame.draw.circle(screen, (0, 200, 255), (knob_x, knob_y), 12)

    # Text
    txt1 = FONT.render(f"Servo Angle: {servo} deg", True, (255, 255, 255))
    txt2 = FONT.render(f"Target RPM: {rpm:.1f}", True, (255, 255, 255))
    txt3 = FONT.render(f"Current RPM: {current_rpm:.1f}", True, (0, 255, 0))
    screen.blit(txt1, (20, 20))
    screen.blit(txt2, (20, 50))
    screen.blit(txt3, (20, 80))

    pygame.display.flip()

pygame.quit()
ser.close()
