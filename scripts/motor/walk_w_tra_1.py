# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time
import numpy as np
import pandas as pd
from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil

# ============================================
# 1. Configuration
# ============================================
CSV_FILE = "20251212_joints_timeseries_1.csv"
FPS = 50.0
INIT_DURATION = 3.0

KP = 0.2
KD = 0.02
TORQUE_LIMIT = 0.4

# ============================================
# 2. Hardware ID Definition (Physical Robot)
# ============================================
# Left Leg IDs
L_HIP_ROLL   = 1
L_HIP_YAW    = 3
L_HIP_PITCH  = 5
L_KNEE       = 7
L_ANK_PITCH  = 11
L_ANK_ROLL   = 13

# Right Leg IDs
R_HIP_ROLL   = 2
R_HIP_YAW    = 4
R_HIP_PITCH  = 6
R_KNEE       = 8
R_ANK_PITCH  = 12
R_ANK_ROLL   = 14

# Bus Grouping for communication
LEFT_BUS_IDS = [L_HIP_ROLL, L_HIP_YAW, L_HIP_PITCH, L_KNEE, L_ANK_PITCH, L_ANK_ROLL]

# ============================================
# 3. Joint Mapping Verification (CRITICAL)
# ============================================
# CSV Header Order (from your image):
# [0] L_Ank_Roll -> [1] L_Ank_Pitch -> [2] L_Knee -> [3] L_Hip_Pitch -> [4] L_Hip_Yaw -> [5] L_Hip_Roll
# [6] R_Ank_Roll -> ...

JOINT_ORDER = [
    # ----- LEFT LEG (CSV Columns 0 ~ 5) -----
    L_ANK_ROLL,   # [Col 0] CSV: leg_left_ankle_roll_joint  <==> HW ID: 13
    L_ANK_PITCH,  # [Col 1] CSV: leg_left_ankle_pitch_joint <==> HW ID: 11
    L_KNEE,       # [Col 2] CSV: leg_left_knee_pitch_joint  <==> HW ID: 7
    L_HIP_PITCH,  # [Col 3] CSV: leg_left_hip_pitch_joint   <==> HW ID: 5
    L_HIP_YAW,    # [Col 4] CSV: leg_left_hip_yaw_joint     <==> HW ID: 3
    L_HIP_ROLL,   # [Col 5] CSV: leg_left_hip_roll_joint    <==> HW ID: 1
    
    # ----- RIGHT LEG (CSV Columns 6 ~ 11) -----
    R_ANK_ROLL,   # [Col 6] CSV: leg_right_ankle_roll_joint <==> HW ID: 14
    R_ANK_PITCH,  # [Col 7] CSV: leg_right_ankle_pitch_joint<==> HW ID: 12
    R_KNEE,       # [Col 8] CSV: leg_right_knee_pitch_joint <==> HW ID: 8
    R_HIP_PITCH,  # [Col 9] CSV: leg_right_hip_pitch_joint  <==> HW ID: 6
    R_HIP_YAW,    # [Col 10] CSV: leg_right_hip_yaw_joint   <==> HW ID: 4
    R_HIP_ROLL    # [Col 11] CSV: leg_right_hip_roll_joint  <==> HW ID: 2
]

# CAN Bus Setup
bus_left = recoil.Bus(channel="can0", bitrate=1000000)
bus_right = recoil.Bus(channel="can1", bitrate=1000000)

def get_bus(device_id):
    if device_id in LEFT_BUS_IDS:
        return bus_left
    return bus_right

def configure_motors():
    print("Configuring motors...")
    unique_ids = set(JOINT_ORDER)
    for device_id in unique_ids:
        bus = get_bus(device_id)
        bus.write_position_kp(device_id, KP)
        bus.write_position_kd(device_id, KD)
        bus.write_torque_limit(device_id, TORQUE_LIMIT)
        bus.set_mode(device_id, recoil.Mode.POSITION)
        bus.feed(device_id)

def load_trajectory(filename):
    try:
        df = pd.read_csv(filename)
        # Remove non-data columns
        cols_to_drop = [c for c in df.columns if 'Index' in c or 'Time' in c]
        if cols_to_drop:
            df = df.drop(columns=cols_to_drop)
        
        traj = df.to_numpy()
        
        # Structure Check
        if traj.shape[1] != len(JOINT_ORDER):
            raise ValueError(f"CSV Column Count ({traj.shape[1]}) != Joint Count ({len(JOINT_ORDER)})")
            
        print(f"Loaded CSV: {traj.shape} (Frames x Joints)")
        return traj
    except Exception as e:
        print(f"[ERROR] {e}")
        return None

def main():
    # ------------------------------------------------
    # 0. Mapping Verification Print
    # ------------------------------------------------
    print("="*50)
    print("      JOINT MAPPING VERIFICATION      ")
    print("="*50)
    print(f"{'CSV Col':<8} | {'CSV Name (Expected)':<25} | {'HW ID':<5}")
    print("-" * 50)
    
    csv_names = [
        "L_Ank_Roll", "L_Ank_Pitch", "L_Knee", "L_Hip_Pitch", "L_Hip_Yaw", "L_Hip_Roll",
        "R_Ank_Roll", "R_Ank_Pitch", "R_Knee", "R_Hip_Pitch", "R_Hip_Yaw", "R_Hip_Roll"
    ]
    
    for i, hw_id in enumerate(JOINT_ORDER):
        print(f"{i:<8} | {csv_names[i]:<25} | {hw_id:<5}")
    print("="*50)
    print("Check if the table above matches your robot configuration.")
    time.sleep(2.0) # Pause for user to check

    # ------------------------------------------------
    # 1. Setup & Load
    # ------------------------------------------------
    rate = RateLimiter(frequency=FPS)
    configure_motors()
    traj_data = load_trajectory(CSV_FILE)
    
    if traj_data is None:
        return

    # Read Current State
    start_pos = np.zeros(len(JOINT_ORDER))
    for i, device_id in enumerate(JOINT_ORDER):
        bus = get_bus(device_id)
        p, _ = bus.write_read_pdo_2(device_id, 0.0, 0.0)
        start_pos[i] = p if p is not None else 0.0

    target_start = traj_data[0]
    
    try:
        # ------------------------------------------------
        # 2. Phase 1: Interpolation (Home -> Start)
        # ------------------------------------------------
        print(f"\n[Phase 1] Moving to Start Pose ({INIT_DURATION}s)...")
        steps = int(INIT_DURATION * FPS)
        
        for s in range(steps):
            alpha = s / float(steps)
            cmd = (1 - alpha) * start_pos + alpha * target_start
            
            for i, device_id in enumerate(JOINT_ORDER):
                bus = get_bus(device_id)
                bus.write_read_pdo_2(device_id, cmd[i], 0.0)
            rate.sleep()
            
        # ------------------------------------------------
        # 3. Phase 2: Trajectory Playback
        # ------------------------------------------------
        print(f"\n[Phase 2] Playing Trajectory...")
        start_time = time.time()
        
        for frame_idx, target_vec in enumerate(traj_data):
            for i, device_id in enumerate(JOINT_ORDER):
                bus = get_bus(device_id)
                bus.write_read_pdo_2(device_id, target_vec[i], 0.0)
            
            if frame_idx % 50 == 0:
                print(f"Frame {frame_idx}/{len(traj_data)}")
            
            rate.sleep()
            
        print("Done.")
        
    except KeyboardInterrupt:
        print("\nInterrupted.")
        
    finally:
        print("Setting IDLE...")
        for device_id in set(JOINT_ORDER):
            get_bus(device_id).set_mode(device_id, recoil.Mode.IDLE)
        bus_left.stop()
        bus_right.stop()

if __name__ == "__main__":
    main()