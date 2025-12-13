# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time
import numpy as np
import pandas as pd
from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil

# ============================================
# [USER PARAMETERS] 제어 및 안전 설정
# ============================================
TRAJECTORY_SCALE = 15.0   # 궤적 크기 비율 (0.0 ~ 1.0 ~ N)
TORQUE_LIMIT     = 2.0   # [CRITICAL] 모터 최대 출력 토크 제한 (단위: Nm 또는 Current Scale)
                         # 공중 테스트 시 0.5~1.0 추천, 실제 보행 시 필요에 따라 증량

KP = 0.2                 # 위치 제어 비례 게인
KD = 0.01                # 위치 제어 미분 게인

# ============================================
# 1. CAN 버스 및 ID 설정
# ============================================
bus_left = recoil.Bus(channel="can0", bitrate=1000000)
bus_right = recoil.Bus(channel="can1", bitrate=1000000)

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
    if device_id in left_leg_ids: return bus_left
    return bus_right

# ============================================
# 2. CSV 궤적 로드
# ============================================
CSV_PATH = "./scripts/motor/20251212_joints_timeseries_1_zero.csv"
try:
    print(f"Loading trajectory from {CSV_PATH}...")
    traj_df = pd.read_csv(CSV_PATH)
    traj_time = traj_df.iloc[:, 0].values
    traj_duration = traj_time[-1]
    print(f"Loaded. Duration: {traj_duration:.2f}s, Scale: {TRAJECTORY_SCALE}, TorqueLimit: {TORQUE_LIMIT}")
except Exception as e:
    print(f"[CRITICAL ERROR] Failed to load CSV: {e}")
    exit(1)

ramp_duration = 5.0
rate = RateLimiter(frequency=50.0)

# ============================================
# 3. 모터 초기화 (토크 리밋 복구됨)
# ============================================
print("Initializing motors...")

# [LEFT LEG CONFIGURATION]
for device_id in left_leg_ids:
    bus_left.write_position_kp(device_id, KP)
    bus_left.write_position_kd(device_id, KD)
    
    # [SAFETY] 토크 리밋 설정 복구
    bus_left.write_torque_limit(device_id, TORQUE_LIMIT)
    
    bus_left.set_mode(device_id, recoil.Mode.POSITION)
    bus_left.feed(device_id)

# [RIGHT LEG CONFIGURATION]
for device_id in right_leg_ids:
    bus_right.write_position_kp(device_id, KP)
    bus_right.write_position_kd(device_id, KD)
    
    # [SAFETY] 토크 리밋 설정 복구
    bus_right.write_torque_limit(device_id, TORQUE_LIMIT)
    
    bus_right.set_mode(device_id, recoil.Mode.POSITION)
    bus_right.feed(device_id)

# 초기 위치 읽기 (Homing)
initial_positions = {}
print("\nReading initial positions...")
time.sleep(0.5)

for device_id in all_ids:
    bus = get_bus(device_id)
    pos, _ = bus.write_read_pdo_2(device_id, 0.0, 0.0)
    if pos is None: pos = 0.0
    initial_positions[device_id] = pos

print("\n" + "="*50)
print(f"Start Tracking | Scale: {TRAJECTORY_SCALE} | Torque Limit: {TORQUE_LIMIT}")
print("="*50)

# ============================================
# 4. 메인 제어 루프
# ============================================
start_time = time.time()

try:
    while True:
        current_time = time.time() - start_time
        if current_time > traj_duration:
            print("End of trajectory.")
            break

        ramp_scale = min(current_time / ramp_duration, 1.0)
        final_scale = TRAJECTORY_SCALE * ramp_scale

        for device_id in all_ids:
            col_name = str(device_id)
            if col_name in traj_df.columns:
                raw_offset = np.interp(current_time, traj_time, traj_df[col_name].values)
            else:
                raw_offset = 0.0

            init_pos = initial_positions[device_id]
            target_angle = init_pos + (raw_offset * final_scale)

            # 명령 전송
            get_bus(device_id).write_read_pdo_2(device_id, target_angle, 0.0)

        rate.sleep()

except KeyboardInterrupt:
    print("\nStopping...")

except Exception as e:
    print(f"\nError: {e}")

finally:
    print("Setting motors to IDLE.")
    for device_id in all_ids:
        get_bus(device_id).set_mode(device_id, recoil.Mode.IDLE)
    bus_left.stop()
    bus_right.stop()