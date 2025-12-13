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
FPS = 20.0                          # 제어 주파수 (Hz)
INIT_DURATION = 3.0                 # 초기 자세 잡는 시간 (초)

# [중요] 궤적 추종을 위한 게인 설정 (기존 0.2보다 높여야 정확히 따라감)
KP = 0.2  
KD = 0.05
TORQUE_LIMIT = 1

# ============================================
# 2. CAN 버스 및 ID 매핑
# ============================================
bus_left = recoil.Bus(channel="can0", bitrate=1000000)
bus_right = recoil.Bus(channel="can1", bitrate=1000000)

# Hardware ID Definitions
L_HIP_ROLL, L_HIP_YAW, L_HIP_PITCH = 1, 3, 5
L_KNEE = 7
L_ANK_PITCH, L_ANK_ROLL = 11, 13

R_HIP_ROLL, R_HIP_YAW, R_HIP_PITCH = 2, 4, 6
R_KNEE = 8
R_ANK_PITCH, R_ANK_ROLL = 12, 14

LEFT_BUS_IDS = [L_HIP_ROLL, L_HIP_YAW, L_HIP_PITCH, L_KNEE, L_ANK_PITCH, L_ANK_ROLL]

# [CRITICAL] CSV 데이터 컬럼 순서와 하드웨어 ID 매핑
# CSV 순서: 발목 -> 엉덩이 (User Image 기준)
JOINT_ORDER = [
    # --- Left Leg (Cols 0-5) ---
    L_ANK_ROLL,     # Col 0 (ID 13)
    L_ANK_PITCH,    # Col 1 (ID 11)
    L_KNEE,         # Col 2 (ID 7)
    L_HIP_PITCH,    # Col 3 (ID 5)
    L_HIP_YAW,      # Col 4 (ID 3)
    L_HIP_ROLL,     # Col 5 (ID 1)
    
    # --- Right Leg (Cols 6-11) ---
    R_ANK_ROLL,     # Col 6 (ID 14)
    R_ANK_PITCH,    # Col 7 (ID 12)
    R_KNEE,         # Col 8 (ID 8)
    R_HIP_PITCH,    # Col 9 (ID 6)
    R_HIP_YAW,      # Col 10 (ID 4)
    R_HIP_ROLL      # Col 11 (ID 2)
]

def get_bus(device_id):
    if device_id in LEFT_BUS_IDS:
        return bus_left
    return bus_right

# ============================================
# 3. 헬퍼 함수
# ============================================
def configure_motors():
    """모터 게인 및 모드 설정"""
    print("Configuring motors...")
    for device_id in set(JOINT_ORDER):
        bus = get_bus(device_id)
        bus.write_position_kp(device_id, KP)
        bus.write_position_kd(device_id, KD)
        bus.write_torque_limit(device_id, TORQUE_LIMIT)
        bus.set_mode(device_id, recoil.Mode.POSITION)
        bus.feed(device_id)

def load_trajectory(filename):
    """CSV 로드 및 Numpy 변환"""
    try:
        df = pd.read_csv(filename)
        # 불필요한 컬럼 제거 (Index, Timestamp 등)
        cols_to_drop = [c for c in df.columns if 'Index' in c or 'Time' in c]
        if cols_to_drop:
            df = df.drop(columns=cols_to_drop)
        
        traj = df.to_numpy()
        print(f"Loaded CSV: {traj.shape} (Frames x Joints)")
        
        if traj.shape[1] != len(JOINT_ORDER):
            print(f"[ERROR] CSV Column count ({traj.shape[1]}) != ID count ({len(JOINT_ORDER)})")
            return None
        return traj
    except Exception as e:
        print(f"[ERROR] Failed to load CSV: {e}")
        return None

# ============================================
# 4. 메인 실행
# ============================================
def main():
    rate = RateLimiter(frequency=FPS)
    
    # 1. 초기화 및 데이터 로드
    configure_motors()
    traj_data = load_trajectory(CSV_FILE)
    if traj_data is None: return

    # 2. 현재 로봇 상태 읽기 (안전한 시작을 위해 필수)
    print("\nReading current robot posture...")
    start_pos_actual = np.zeros(len(JOINT_ORDER))
    
    for i, device_id in enumerate(JOINT_ORDER):
        bus = get_bus(device_id)
        # 현재 위치 읽기 (목표값 0을 보내지만, 즉시 반영되진 않음)
        pos, _ = bus.write_read_pdo_2(device_id, 0.0, 0.0)
        start_pos_actual[i] = pos if pos is not None else 0.0

    target_start_pos = traj_data[0] # CSV의 첫 번째 프레임

    print(f"Ready. Press Ctrl+C to stop.")
    time.sleep(1.0)

    try:
        # ---------------------------------------------------------
        # PHASE 1: Soft Start (Interpolation)
        # 현재 위치 -> CSV 첫 프레임으로 부드럽게 이동
        # ---------------------------------------------------------
        print(f"\n[Phase 1] Moving to Start Position ({INIT_DURATION}s)...")
        init_steps = int(INIT_DURATION * FPS)
        
        for step in range(init_steps):
            alpha = step / float(init_steps) # 0.0 -> 1.0
            
            # 보간 계산: (1-a)*현재 + a*목표
            current_cmd = (1 - alpha) * start_pos_actual + alpha * target_start_pos
            
            # 명령 전송
            for i, device_id in enumerate(JOINT_ORDER):
                bus = get_bus(device_id)
                bus.write_read_pdo_2(device_id, current_cmd[i], 0.0)
            
            rate.sleep()

        print("[Phase 1] Complete. Starting Trajectory...")
        time.sleep(0.5)

        # ---------------------------------------------------------
        # PHASE 2: CSV Trajectory Playback
        # ---------------------------------------------------------
        print(f"\n[Phase 2] Playing CSV Data...")
        
        for frame_idx, joint_targets in enumerate(traj_data):
            # joint_targets는 CSV의 한 줄 (numpy array)
            
            for i, device_id in enumerate(JOINT_ORDER):
                bus = get_bus(device_id)
                # 매핑된 ID로 목표 위치 전송
                bus.write_read_pdo_2(device_id, joint_targets[i], 0.0)
            
            # 진행 상황 출력 (1초마다)
            if frame_idx % 50 == 0:
                print(f"Frame {frame_idx}/{len(traj_data)}")

            rate.sleep()

        print("Trajectory Finished.")

    except KeyboardInterrupt:
        print("\nStopping by user...")

    except Exception as e:
        print(f"\nRuntime Error: {e}")

    finally:
        # ---------------------------------------------------------
        # 5. 안전 종료 (IDLE 모드)
        # ---------------------------------------------------------
        print("Setting all motors to IDLE.")
        for device_id in set(JOINT_ORDER):
            bus = get_bus(device_id)
            bus.set_mode(device_id, recoil.Mode.IDLE)
        
        bus_left.stop()
        bus_right.stop()
        print("Buses stopped.")

if __name__ == "__main__":
    main()