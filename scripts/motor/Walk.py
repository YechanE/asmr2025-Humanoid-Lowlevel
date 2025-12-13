# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers. 

import time
import numpy as np

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel. recoil as recoil


# ============================================
# CAN 버스 설정 (왼쪽: can0, 오른쪽: can1)
# ============================================
bus_left = recoil. Bus(channel="can0", bitrate=1000000)   # 왼쪽 다리
bus_right = recoil.Bus(channel="can1", bitrate=1000000)  # 오른쪽 다리

# ============================================
# Joint ID Mapping (CAN ID 기준)
# ============================================
# Left Leg (CAN0)
LEFT_HIP_ROLL = 1      # left_hip_roll (Joint 10)
LEFT_HIP_YAW = 3       # left_hip_yaw (Joint 11)
LEFT_HIP_PITCH = 5     # left_hip_pitch (Joint 12)
LEFT_KNEE_PITCH = 7    # left_knee_pitch (Joint 13)
LEFT_ANKLE_PITCH = 11  # left_ankle_pitch (Joint 14)
LEFT_ANKLE_ROLL = 13   # left_ankle_roll (Joint 15)

# Right Leg (CAN1)
RIGHT_HIP_ROLL = 2     # right_hip_roll (Joint 16)
RIGHT_HIP_YAW = 4      # right_hip_yaw (Joint 17)
RIGHT_HIP_PITCH = 6    # right_hip_pitch (Joint 18)
RIGHT_KNEE_PITCH = 8   # right_knee_pitch (Joint 19)
RIGHT_ANKLE_PITCH = 12 # right_ankle_pitch (Joint 20)
RIGHT_ANKLE_ROLL = 14  # right_ankle_roll (Joint 21)

# 모든 모터 ID
left_leg_ids = [LEFT_HIP_ROLL, LEFT_HIP_YAW, LEFT_HIP_PITCH, 
                LEFT_KNEE_PITCH, LEFT_ANKLE_PITCH, LEFT_ANKLE_ROLL]
right_leg_ids = [RIGHT_HIP_ROLL, RIGHT_HIP_YAW, RIGHT_HIP_PITCH, 
                 RIGHT_KNEE_PITCH, RIGHT_ANKLE_PITCH, RIGHT_ANKLE_ROLL]

# 디바이스 ID와 버스 매핑
def get_bus(device_id):
    """디바이스 ID에 따라 적절한 CAN 버스 반환"""
    if device_id in left_leg_ids:
        return bus_left
    else:
        return bus_right

# ============================================
# 걷기 모션 파라미터 (라디안 단위)
# ============================================
# 각 조인트별 진폭 설정 (걷기 모션에 적합하게 조정)
WALK_AMPLITUDES = {
    # Left Leg
    LEFT_HIP_ROLL: 0.05,      # 작은 롤 움직임
    LEFT_HIP_YAW: 0.0,        # yaw는 거의 움직이지 않음
    LEFT_HIP_PITCH: 0.3,      # 걷기의 주요 움직임
    LEFT_KNEE_PITCH: 0.4,     # 무릎 굽힘
    LEFT_ANKLE_PITCH: 0.2,    # 발목 pitch
    LEFT_ANKLE_ROLL: 0.05,    # 작은 롤 움직임
    
    # Right Leg
    RIGHT_HIP_ROLL: 0.05,
    RIGHT_HIP_YAW: 0.0,
    RIGHT_HIP_PITCH: 0.3,
    RIGHT_KNEE_PITCH: 0.4,
    RIGHT_ANKLE_PITCH: 0.2,
    RIGHT_ANKLE_ROLL: 0.05,
}

# 각 조인트별 위상 오프셋 (0 ~ 2*pi)
PHASE_OFFSETS = {
    # Left Leg - 기준 위상 (0)
    LEFT_HIP_ROLL: 0.0,
    LEFT_HIP_YAW: 0.0,
    LEFT_HIP_PITCH: 0.0,
    LEFT_KNEE_PITCH: np.pi / 4,      # hip보다 약간 늦게
    LEFT_ANKLE_PITCH: np.pi / 2,     # knee보다 약간 늦게
    LEFT_ANKLE_ROLL: 0.0,
    
    # Right Leg - 180도 위상차 (반대 다리)
    RIGHT_HIP_ROLL: np.pi,
    RIGHT_HIP_YAW: np.pi,
    RIGHT_HIP_PITCH: np.pi,
    RIGHT_KNEE_PITCH: np.pi + np.pi / 4,
    RIGHT_ANKLE_PITCH: np.pi + np. pi / 2,
    RIGHT_ANKLE_ROLL: np.pi,
}

# 방향 계수 (일부 조인트는 반대 방향으로 움직여야 함)
DIRECTION = {
    LEFT_HIP_ROLL: 1,
    LEFT_HIP_YAW: 1,
    LEFT_HIP_PITCH: 1,
    LEFT_KNEE_PITCH: 1,
    LEFT_ANKLE_PITCH: -1,
    LEFT_ANKLE_ROLL: 1,
    
    RIGHT_HIP_ROLL: -1,
    RIGHT_HIP_YAW: 1,
    RIGHT_HIP_PITCH: 1,
    RIGHT_KNEE_PITCH: 1,
    RIGHT_ANKLE_PITCH: -1,
    RIGHT_ANKLE_ROLL: -1,
}

# gains
kp = 0.2
kd = 0.01

# 걷기 주파수 (Hz)
walk_frequency = 1  # 1 Hz = 1초에 한 걸음

rate = RateLimiter(frequency=200.0)

# ============================================
# 모터 초기화
# ============================================
print("Initializing motors...")
print("Left leg on CAN0, Right leg on CAN1")

# 왼쪽 다리 초기화 (can0)
for device_id in left_leg_ids:
    bus_left.write_position_kp(device_id, kp)
    bus_left.write_position_kd(device_id, kd)
    bus_left.write_torque_limit(device_id, 0.8)
    bus_left.set_mode(device_id, recoil.Mode.POSITION)
    bus_left. feed(device_id)

# 오른쪽 다리 초기화 (can1)
for device_id in right_leg_ids:
    bus_right.write_position_kp(device_id, kp)
    bus_right.write_position_kd(device_id, kd)
    bus_right.write_torque_limit(device_id, 0.8)
    bus_right.set_mode(device_id, recoil.Mode.POSITION)
    bus_right. feed(device_id)

# 각 모터별 초기 위치 읽기
initial_positions = {}

print("\nReading initial positions...")

# 왼쪽 다리 초기 위치 읽기
print("[CAN0 - Left Leg]")
for device_id in left_leg_ids:
    initial_position, _ = bus_left. write_read_pdo_2(device_id, 0.0, 0.0)
    if initial_position is None:
        print(f"  Failed to read initial position for ID {device_id}.  Use 0. 0 rad.")
        initial_position = 0.0
    initial_positions[device_id] = initial_position
    print(f"  ID {device_id:2d}: Initial Position = {initial_position:.3f} rad")

# 오른쪽 다리 초기 위치 읽기
print("[CAN1 - Right Leg]")
for device_id in right_leg_ids:
    initial_position, _ = bus_right.write_read_pdo_2(device_id, 0.0, 0.0)
    if initial_position is None:
        print(f"  Failed to read initial position for ID {device_id}. Use 0. 0 rad.")
        initial_position = 0.0
    initial_positions[device_id] = initial_position
    print(f"  ID {device_id:2d}: Initial Position = {initial_position:.3f} rad")

print("\n" + "=")
