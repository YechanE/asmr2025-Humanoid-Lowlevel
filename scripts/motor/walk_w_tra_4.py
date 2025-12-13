# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time
import numpy as np
import pandas as pd
from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil

# ============================================
# 1. 설정 (Configuration)
# ============================================
CSV_FILE = "20251212_joints_timeseries_1.csv"
FPS = 50.0  # CSV 데이터의 주파수 (Hz)

# 제어 게인 (CSV 궤적을 잘 따라가도록 설정)
KP = 4.0
KD = 0.1
TORQUE_LIMIT = 3.0

# ============================================
# 2. CAN 버스 및 ID 설정
# ============================================
bus_left = recoil.Bus(channel="can0", bitrate=1000000)
bus_right = recoil.Bus(channel="can1", bitrate=1000000)

# Joint ID Definitions
L_HIP_ROLL, L_HIP_YAW, L_HIP_PITCH = 1, 3, 5
L_KNEE = 7
L_ANK_PITCH, L_ANK_ROLL = 11, 13

R_HIP_ROLL, R_HIP_YAW, R_HIP_PITCH = 2, 4, 6
R_KNEE = 8
R_ANK_PITCH, R_ANK_ROLL = 12, 14

LEFT_BUS_IDS = [L_HIP_ROLL, L_HIP_YAW, L_HIP_PITCH, L_KNEE, L_ANK_PITCH, L_ANK_ROLL]

# [중요] CSV 데이터 컬럼 순서와 하드웨어 ID 매핑 (이전 분석 기반)
JOINT_ORDER = [
    L_ANK_ROLL, L_ANK_PITCH, L_KNEE, L_HIP_PITCH, L_HIP_YAW, L_HIP_ROLL,
    R_ANK_ROLL, R_ANK_PITCH, R_KNEE, R_HIP_PITCH, R_HIP_YAW, R_HIP_ROLL
]

def get_bus(device_id):
    if device_id in LEFT_BUS_IDS:
        return bus_left
    return bus_right

# ============================================
# 3. 데이터 로드 및 초기화
# ============================================
def load_trajectory(filename):
    try:
        df = pd.read_csv(filename)
        # 불필요한 컬럼 제거
        cols_to_drop = [c for c in df.columns if 'Index' in c or 'Time' in c]
        if cols_to_drop:
            df = df.drop(columns=cols_to_drop)
        return df.to_numpy()
    except Exception as e:
        print(f"Error loading CSV: {e}")
        return None

# 모터 설정
print("Initializing motors...")
for device_id in set(JOINT_ORDER):
    bus = get_bus(device_id)
    bus.write_position_kp(device_id, KP)
    bus.write_position_kd(device_id, KD)
    bus.write_torque_limit(device_id, TORQUE_LIMIT)
    bus.set_mode(device_id, recoil.Mode.POSITION)
    bus.feed(device_id)

# CSV 로드
traj_data = load_trajectory(CSV_FILE)
if traj_data is None:
    exit()

# [핵심] 현재 위치 읽기 (Zero Reference 설정)
print("\nReading initial positions...")
start_pos_actual = np.zeros(len(JOINT_ORDER))

for i, device_id in enumerate(JOINT_ORDER):
    bus = get_bus(device_id)
    # 현재 값 읽기 (목표값 0.0은 무시됨)
    pos, _ = bus.write_read_pdo_2(device_id, 0.0, 0.0)
    start_pos_actual[i] = pos if pos is not None else 0.0

# [핵심] 오프셋 계산: (현재 실제 위치) - (CSV 첫 프레임)
# 이 오프셋을 더해주면 CSV의 0번째 프레임은 정확히 '현재 위치'가 됩니다.
csv_start_frame = traj_data[0]
position_offset = start_pos_actual - csv_start_frame

print(f"Offset calibrated. Starting IMMEDIATELY from current pose.")
print("="*50)

# ============================================
# 4. 메인 제어 루프 (즉시 실행)
# ============================================
rate = RateLimiter(frequency=FPS)

try:
    # CSV 데이터를 순차적으로 실행
    for frame_idx, csv_row in enumerate(traj_data):
        
        # [Relative Target Calculation]
        # 목표 위치 = CSV 데이터 + 오프셋
        # (시작 시점에서는 'CSV[0] + (Current - CSV[0]) = Current'가 되므로 튀지 않음)
        target_cmds = csv_row + position_offset
        
        for i, device_id in enumerate(JOINT_ORDER):
            bus = get_bus(device_id)
            # 매핑된 ID로 명령 전송
            bus.write_read_pdo_2(device_id, target_cmds[i], 0.0)
        
        # 디버깅 (50프레임마다)
        if frame_idx % 50 == 0:
            print(f"Frame {frame_idx}/{len(traj_data)}")

        rate.sleep()

    print("Trajectory Finished.")

except KeyboardInterrupt:
    print("\nStopping...")

except Exception as e:
    print(f"\nError: {e}")

finally:
    # 종료 처리
    print("Setting IDLE mode.")
    for device_id in set(JOINT_ORDER):
        bus = get_bus(device_id)
        bus.set_mode(device_id, recoil.Mode.IDLE)
    
    bus_left.stop()
    bus_right.stop()
    print("Buses stopped.")