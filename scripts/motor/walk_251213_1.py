# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time
import numpy as np
from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil

# ============================================
# 1. CAN 버스 및 ID 설정
# ============================================
bus_left = recoil.Bus(channel="can0", bitrate=1000000)   # 왼쪽 다리
bus_right = recoil.Bus(channel="can1", bitrate=1000000)  # 오른쪽 다리

# Joint ID Definitions
LEFT_HIP_ROLL, LEFT_HIP_YAW, LEFT_HIP_PITCH = 1, 3, 5
LEFT_KNEE_PITCH = 7
LEFT_ANKLE_PITCH, LEFT_ANKLE_ROLL = 11, 13

RIGHT_HIP_ROLL, RIGHT_HIP_YAW, RIGHT_HIP_PITCH = 2, 4, 6
RIGHT_KNEE_PITCH = 8
RIGHT_ANKLE_PITCH, RIGHT_ANKLE_ROLL = 12, 14

left_leg_ids = [LEFT_HIP_ROLL, LEFT_HIP_YAW, LEFT_HIP_PITCH, 
                LEFT_KNEE_PITCH, LEFT_ANKLE_PITCH, LEFT_ANKLE_ROLL]
right_leg_ids = [RIGHT_HIP_ROLL, RIGHT_HIP_YAW, RIGHT_HIP_PITCH, 
                 RIGHT_KNEE_PITCH, RIGHT_ANKLE_PITCH, RIGHT_ANKLE_ROLL]

all_ids = left_leg_ids + right_leg_ids

def get_bus(device_id):
    """ID에 따라 올바른 버스 객체 반환"""
    if device_id in left_leg_ids:
        return bus_left
    return bus_right

# ============================================
# 2. 보행 모션 파라미터 (User Provided)
# ============================================
WALK_AMPLITUDES = {
    LEFT_HIP_ROLL: 0.0, LEFT_HIP_YAW: 0.0, LEFT_HIP_PITCH: 0.6,
    LEFT_KNEE_PITCH: 0.6, LEFT_ANKLE_PITCH: 0.2, LEFT_ANKLE_ROLL: 0.0,
    
    RIGHT_HIP_ROLL: 0.0, RIGHT_HIP_YAW: 0.0, RIGHT_HIP_PITCH: 0.6,
    RIGHT_KNEE_PITCH: 0.6, RIGHT_ANKLE_PITCH: 0.2, RIGHT_ANKLE_ROLL: 0.0,
}

PHASE_OFFSETS = {
    LEFT_HIP_ROLL: 0.0, LEFT_HIP_YAW: 0.0, LEFT_HIP_PITCH: 0.0,
    LEFT_KNEE_PITCH: np.pi / 4, LEFT_ANKLE_PITCH: np.pi / 2, LEFT_ANKLE_ROLL: 0.0,
    
    # Right Leg (π offset for anti-phase)
    RIGHT_HIP_ROLL: np.pi, RIGHT_HIP_YAW: np.pi, RIGHT_HIP_PITCH: np.pi,
    RIGHT_KNEE_PITCH: np.pi + np.pi / 4, RIGHT_ANKLE_PITCH: np.pi + np.pi / 2,
    RIGHT_ANKLE_ROLL: np.pi,
}

DIRECTION = {
    LEFT_HIP_ROLL: 1, LEFT_HIP_YAW: 1, LEFT_HIP_PITCH: 1,
    LEFT_KNEE_PITCH: 1, LEFT_ANKLE_PITCH: -1, LEFT_ANKLE_ROLL: 1,
    
    RIGHT_HIP_ROLL: -1, RIGHT_HIP_YAW: -1, RIGHT_HIP_PITCH: -1,
    RIGHT_KNEE_PITCH: -1, RIGHT_ANKLE_PITCH: 1, RIGHT_ANKLE_ROLL: -1,
}

# Control Parameters
kp = 1
kd = 0.015
walk_frequency = 1.0  # 1 Hz

# Soft Start Parameters (안전 장치)
ramp_duration = 5.0   # 5초 동안 서서히 진폭 증가

rate = RateLimiter(frequency=200.0)

# ============================================
# 3. 모터 초기화 및 초기 위치 읽기
# ============================================
print("Initializing motors...")

# Configure Left Leg
for device_id in left_leg_ids:
    bus_left.write_position_kp(device_id, kp)
    time.sleep(0.01)
    bus_left.write_position_kd(device_id, kd)
    time.sleep(0.01)
    bus_left.write_torque_limit(device_id, 2.0)
    time.sleep(0.01)
    bus_left.set_mode(device_id, recoil.Mode.POSITION)
    time.sleep(0.01)
    bus_left.feed(device_id)

# Configure Right Leg
for device_id in right_leg_ids:
    bus_right.write_position_kp(device_id, kp)
    time.sleep(0.01)
    bus_right.write_position_kd(device_id, kd)
    time.sleep(0.01)
    bus_right.write_torque_limit(device_id, 2.0)
    time.sleep(0.01)
    bus_right.set_mode(device_id, recoil.Mode.POSITION)
    time.sleep(0.01)
    bus_right.feed(device_id)

# Read Initial Positions
initial_positions = {}
print("\nReading initial positions (Homing)...")
time.sleep(0.5) # 통신 안정화 대기

for device_id in all_ids:
    bus = get_bus(device_id)
    pos, _ = bus.write_read_pdo_2(device_id, 0.0, 0.0)
    
    if pos is None:
        print(f"[WARNING] Failed ID {device_id}. Assuming 0.0.")
        pos = 0.0
    
    initial_positions[device_id] = pos
    print(f"ID {device_id:2d}: Init Pos = {pos:.3f} rad")

print("\n" + "="*50)
print("Starting Walking Motion (Soft Start Active)")
print(" Amplitude will ramp up over 5 seconds.")
print(" Press Ctrl+C to Stop.")
print("="*50)

# ============================================
# 4. 메인 제어 루프
# ============================================
start_time = time.time()

try:
    while True:
        # 현재 시간 및 램프 계수 계산
        current_time = time.time() - start_time
        
        # Ramping: 0.0에서 1.0까지 ramp_duration 동안 선형 증가
        ramp_scale = min(current_time / ramp_duration, 1.0)
        
        # 기본 사인파 계산 (위상은 개별 모터에서 적용)
        # omega * t
        phase_base = 2 * np.pi * walk_frequency * current_time

        for device_id in all_ids:
            # 1. 파라미터 가져오기
            bus = get_bus(device_id)
            init_pos = initial_positions[device_id]
            amp = WALK_AMPLITUDES.get(device_id, 0.0)
            phi = PHASE_OFFSETS.get(device_id, 0.0)
            direction = DIRECTION.get(device_id, 1)

            # 2. 목표 위치 계산
            # Target = Init + (Amp * Ramp * sin(wt + phi) * Dir)
            sine_val = np.sin(phase_base + phi)
            offset = amp * ramp_scale * sine_val * direction
            
            target_angle = init_pos + offset
            print(f"ID {device_id} | Target Angle: {target_angle:.3f} rad")

            # 3. 명령 전송
            measured_pos, measured_vel = bus.write_read_pdo_2(
                device_id, target_angle, 0.0
            )
            
            # 4. 디버깅 출력 (ID 1번, 2번만 출력하여 화면 도배 방지)
            if device_id in [LEFT_HIP_PITCH, RIGHT_HIP_PITCH] and measured_pos is not None:
                 leg_name = "L" if device_id == LEFT_HIP_PITCH else "R"
                 # print(f"[{leg_name}_HIP] Tgt:{target_angle:.3f} | Pos:{measured_pos:.3f} | Ramp:{ramp_scale:.2f}")

        rate.sleep()

except KeyboardInterrupt:
    print("\nStopping...")

except Exception as e:
    print(f"\nError: {e}")

finally:
    # 5. 안전 종료 (IDLE 모드)
    print("Setting all motors to IDLE.")
    for device_id in all_ids:
        bus = get_bus(device_id)
        bus.set_mode(device_id, recoil.Mode.IDLE)
    
    bus_left.stop()
    bus_right.stop()
    print("Buses stopped.")