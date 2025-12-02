import serial
import time

PORT = "/dev/ttyUSB0"  # 실제 포트로 바꾸기 (예: /dev/ttyUSB1, /dev/ttyACM0 등)
BAUDS = [4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800]

print(f"Scanning port {PORT}...")

for b in BAUDS:
    try:
        ser = serial.Serial(PORT, b, timeout=0.2)
    except Exception as e:
        print(f"[{b}] open fail: {e}")
        continue

    total = 0
    header55 = 0
    t0 = time.time()
    while time.time() - t0 < 1.0:
        n = ser.in_waiting
        if n:
            chunk = ser.read(n)
            total += len(chunk)
            header55 += chunk.count(0x55)
    ser.close()
    print(f"[{b:7d}] bytes={total:5d}, 0x55_count={header55:5d}")
