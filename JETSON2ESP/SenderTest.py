import serial
import time

PORT = "COM5"  # Windows 示例，换成你实际看到的端口
BAUDRATE = 115200

ser = serial.Serial(PORT, BAUDRATE, timeout=1)
time.sleep(2)

print("Connected")

periods = [2000, 200]

while True:
    for p in periods:
        ser.write(f"{p}\n".encode("utf-8"))
        print("Sent:", p)
        time.sleep(6)
