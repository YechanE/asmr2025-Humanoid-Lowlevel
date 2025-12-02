import serial
import time
import sys

def scan_baudrates(port="/dev/ttyUSB0"):
    # 테스트할 Baudrate 목록
    rates = [115200, 460800, 9600]
    
    print(f"--- Starting Raw Data Diagnostic on {port} ---")
    
    for baud in rates:
        print(f"\n[Testing Baudrate: {baud}]")
        try:
            with serial.Serial(port, baud, timeout=0.1) as ser:
                # 포트 열고 잠시 대기 (노이즈 플러싱)
                time.sleep(0.1)
                ser.reset_input_buffer()
                
                start_time = time.time()
                received_bytes = bytearray()
                
                # 2초간 데이터 수집
                while time.time() - start_time < 2.0:
                    if ser.in_waiting > 0:
                        chunk = ser.read(ser.in_waiting)
                        received_bytes.extend(chunk)
                    time.sleep(0.01)
                
                if len(received_bytes) == 0:
                    print(" -> RESULT: NO DATA RECEIVED (Silence)")
                    print("    Possibilities: RX/TX wiring, Power issue, Device is idle.")
                else:
                    print(f" -> RESULT: {len(received_bytes)} bytes received.")
                    # 앞부분 20바이트를 Hex로 출력
                    hex_str = " ".join([f"{b:02X}" for b in received_bytes[:30]])
                    print(f"    Raw Dump (First 30 bytes): {hex_str}")
                    
                    # 0x55 헤더 존재 여부 확인
                    if 0x55 in received_bytes:
                        print("    [PASS] Header 0x55 found in stream.")
                    else:
                        print("    [FAIL] No 0x55 header found. Data is garbage (wrong baudrate?).")

        except serial.SerialException as e:
            print(f" -> Error opening port: {e}")

if __name__ == "__main__":
    scan_baudrates()