# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time
import struct
import threading
import serial
import numpy as np
from loop_rate_limiters import RateLimiter

class ImuRegisters:
    """Register address mapping for the IMU."""
    SAVE = 0x00
    RSW = 0x02
    RRATE = 0x03
    BAUD = 0x04

class FrameType:
    ACCELERATION = 0x51
    ANGULAR_VELOCITY = 0x52
    ANGLE = 0x53
    QUATERNION = 0x59

class Baudrate:
    BAUD_115200 = 0x06
    BAUD_460800 = 0x08

class RobustSerialImu:
    """
    Thread-safe, buffered driver for HiWonder IM10A IMU.
    Implements locking and stream-based parsing to prevent data tearing and desync.
    """
    FRAME_LENGTH = 11
    HEADER = 0x55

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 9600):
        self.port = port
        self.baud = baudrate
        self.lock = threading.Lock()
        self.is_stopped = threading.Event()
        
        # Try to open serial port
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            print(f"[System] Serial opened on {port} at {baudrate} baud.")
        except serial.SerialException as e:
            print(f"[Error] Failed to open serial port: {e}")
            raise

        # Data containers (Protected by Lock)
        self._acc = np.zeros(3, dtype=np.float32)
        self._gyro = np.zeros(3, dtype=np.float32)
        self._quat = np.zeros(4, dtype=np.float32)
        self._angle = np.zeros(3, dtype=np.float32)

        # Internal buffer for stream parsing
        self.buffer = bytearray()

    def start(self):
        """Starts the background reader thread."""
        self.is_stopped.clear()
        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.thread.start()
        print("[System] IMU reader thread started.")

    def stop(self):
        """Stops the reader thread and closes serial."""
        self.is_stopped.set()
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.ser.is_open:
            self.ser.close()
        print("[System] IMU stopped.")

    def _reader_loop(self):
        """
        Main loop for reading and parsing data.
        Uses a buffer to handle fragmented packets and ensure sync.
        """
        while not self.is_stopped.is_set():
            try:
                # Read all available bytes
                waiting = self.ser.in_waiting
                if waiting > 0:
                    data = self.ser.read(waiting)
                    self.buffer.extend(data)

                # Process buffer while we have at least one full frame
                while len(self.buffer) >= self.FRAME_LENGTH:
                    # Check for Header
                    if self.buffer[0] != self.HEADER:
                        # Slide window by 1 byte until 0x55 is found
                        del self.buffer[0]
                        continue

                    # Extract candidate frame
                    frame = self.buffer[:self.FRAME_LENGTH]
                    
                    # Validate Checksum
                    # Checksum is sum of first 10 bytes (0~9) & 0xFF
                    calc_sum = sum(frame[:10]) & 0xFF
                    received_sum = frame[10]

                    if calc_sum != received_sum:
                        # Invalid frame, discard header and retry
                        # print(f"[Warn] Checksum fail: {calc_sum:#x} != {received_sum:#x}")
                        del self.buffer[0]
                        continue

                    # Valid Frame Found -> Parse it
                    self._parse_frame(frame)
                    
                    # Remove processed frame from buffer
                    del self.buffer[:self.FRAME_LENGTH]

            except Exception as e:
                print(f"[Error] Reader loop exception: {e}")
                time.sleep(0.1)
            
            # Small sleep to prevent CPU hogging if buffer is empty
            time.sleep(0.001)

    def _parse_frame(self, frame):
        frame_type = frame[1]
        data_bytes = frame[2:10]
        data = struct.unpack("<hhhh", data_bytes)

        with self.lock:
            if frame_type == FrameType.ACCELERATION:
                self._acc[0] = data[0] * 16.0 / 32768.0
                self._acc[1] = data[1] * 16.0 / 32768.0
                self._acc[2] = data[2] * 16.0 / 32768.0
            elif frame_type == FrameType.ANGULAR_VELOCITY:
                self._gyro[0] = data[0] * 2000.0 / 32768.0
                self._gyro[1] = data[1] * 2000.0 / 32768.0
                self._gyro[2] = data[2] * 2000.0 / 32768.0
            elif frame_type == FrameType.QUATERNION:
                self._quat[0] = data[0] / 32768.0
                self._quat[1] = data[1] / 32768.0
                self._quat[2] = data[2] / 32768.0
                self._quat[3] = data[3] / 32768.0
            elif frame_type == FrameType.ANGLE:
                self._angle[0] = data[0] * 180.0 / 32768.0
                self._angle[1] = data[1] * 180.0 / 32768.0
                self._angle[2] = data[2] * 180.0 / 32768.0

    # --- Thread-Safe Getters ---
    @property
    def acceleration(self):
        with self.lock:
            return self._acc.copy()

    @property
    def angular_velocity(self):
        with self.lock:
            return self._gyro.copy()

    @property
    def quaternion(self):
        with self.lock:
            return self._quat.copy()
    
    @property
    def euler_angle(self):
        with self.lock:
            return self._angle.copy()

    # --- Configuration Methods ---
    def unlock(self):
        self._send_cmd(0x69, 0xB588)

    def save(self):
        self._send_cmd(0x00, 0x0000)

    def set_sampling_rate(self, rate_hex):
        self._send_cmd(ImuRegisters.RRATE, rate_hex)

    def set_output_content(self, acc=True, gyro=True, quat=True, angle=False):
        content = 0x00
        if acc: content |= 0x02
        if gyro: content |= 0x04
        if angle: content |= 0x08
        if quat: content |= 0x200 # 9th bit
        self._send_cmd(ImuRegisters.RSW, content)

    def _send_cmd(self, reg, val):
        # Packet: 0xFF 0xAA [REG] [VAL_L] [VAL_H]
        payload = struct.pack("<BBBH", 0xFF, 0xAA, reg, val)
        self.ser.write(payload)
        # time.sleep(0.05) # Short delay for device processing

def configure_imu(imu_instance):
    """Runs the configuration sequence."""
    print("[Config] Unlocking device...")
    imu_instance.unlock()
    time.sleep(0.1)

    print("[Config] Setting output content (Acc+Gyro+Quat)...")
    imu_instance.set_output_content(acc=True, gyro=True, quat=True)
    time.sleep(0.1)

    print("[Config] Setting rate to 100Hz...")
    imu_instance.set_sampling_rate(0x09) # 100Hz
    time.sleep(0.1)

    print("[Config] Saving settings...")
    imu_instance.save()
    time.sleep(0.5)
    print("[Config] Done.")

if __name__ == "__main__":
    # 1. Initialize Driver
    # NOTE: baudrate must match the device's current setting. 
    # Factory default is often 115200. If you set it to 460800 previously, change this.
    imu = RobustSerialImu(port="/dev/ttyUSB0", baudrate=9600)
    
    # 2. Configure (Only needed once, but safe to run every time)
    try:
        configure_imu(imu)
    except Exception as e:
        print(f"[Warn] Config failed (Device might be sleeping or wrong baud): {e}")

    # 3. Start Reading
    imu.start()

    # 4. Main Control Loop
    rate = RateLimiter(100) # 100Hz Loop
    
    print("\n[Main] Starting Data Loop. Press Ctrl+C to stop.\n")
    try:
        while True:
            acc = imu.acceleration
            gyro = imu.angular_velocity
            quat = imu.quaternion
            
            # Check if data is actually coming in (not all zeros)
            # Using a small epsilon because exact 0.00 is suspicious for sensors
            if np.all(np.abs(acc) < 0.001) and np.all(np.abs(quat) < 0.001):
                status = "NO DATA / IDLE"
            else:
                status = "ACTIVE"

            print(f"[{status}] "
                  f"Acc: {acc[0]:5.2f} {acc[1]:5.2f} {acc[2]:5.2f} | "
                  f"Gyr: {gyro[0]:5.2f} {gyro[1]:5.2f} {gyro[2]:5.2f} | "
                  f"Quat: {quat[0]:.2f} {quat[1]:.2f} {quat[2]:.2f} {quat[3]:.2f}", 
                  end="\r")
            
            rate.sleep()

    except KeyboardInterrupt:
        print("\n[Main] Interrupted.")
    finally:
        imu.stop()