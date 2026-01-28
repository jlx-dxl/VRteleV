import serial
import time

PORT = "/dev/ttyUSB0"   # Jetson 示例；Windows 用 COMx
BAUDRATE = 115200

ser = serial.Serial(PORT, BAUDRATE, timeout=1)
time.sleep(2)  # 等 ESP32 复位

print("Connected")

# (周期ms, 持续时间ms)
commands = [
    (1000, 5000),  # 慢闪 5 秒
    (500, 5000),   # 中等速度 5 秒
    (200, 5000),   # 快闪 5 秒
]

while True:
    for period, duration in commands:
        cmd = f"{period} {duration}\n"   # 协议: 用空格分隔周期和持续时间
        ser.write(cmd.encode("utf-8"))
        print("Sent:", cmd.strip())
        time.sleep(duration / 1000)
