# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time
import numpy as np
import pandas as pd
from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil

# ============================================
# 1. Configuration & Parameters
# ============================================
CSV_FILE = "20251212_joints_timeseries_1.csv"
FPS = 50.0                          # Control loop frequency (Hz)
INIT_DURATION = 3.0                 # Duration to move from current pose to start pose (sec)

# Motor Control Gains (Trajectory tracking requires stiffer gains than simple walking)
# 기존 0.2는 너무 부드러워서 궤적을 못 따라갈 수 있으므로 약간 상향 조정합니다.
KP = 3.0  
KD = 0.05
TORQUE_LIMIT = 2.0  # Nm

# ============================================
# 2. CAN Bus & Joint Mapping Setup
# ============================================
bus_left = recoil.Bus(channel="can0", bitrate=1000000)
bus_right = recoil.Bus(channel="can1", bitrate=1000000)

# Joint IDs
LEFT_IDS = [1, 3, 5, 7, 11, 13]  # [HipRoll, HipYaw, HipPitch, Knee, AnkPitch, AnkRoll]
RIGHT_IDS = [2, 4, 6, 8, 12, 14] # [HipRoll, HipYaw, HipPitch, Knee, AnkPitch, AnkRoll]

# IMPORTANT: This list determines the mapping from CSV columns to CAN IDs.
# Assuming CSV columns are ordered: [Left Leg (6), Right Leg (6)]
JOINT_ORDER = LEFT_IDS + RIGHT_IDS 

def get_bus(device_id):
    if device_id in LEFT_IDS:
        return bus_left
    return bus_right

# ============================================
# 3. Helper Functions
# ============================================
def configure_motors():
    """Set KP, KD, Torque Limit and Mode for all motors."""
    print("Configuring motors...")
    for device_id in JOINT_ORDER:
        bus = get_bus(device_id)
        bus.write_position_kp(device_id, KP)
        bus.write_position_kd(device_id, KD)
        bus.write_torque_limit(device_id, TORQUE_LIMIT)
        bus.set_mode(device_id, recoil.Mode.POSITION)
        bus.feed(device_id)

def read_current_positions():
    """Read current positions from all motors to ensure safe start."""
    current_pos = np.zeros(len(JOINT_ORDER))
    print("Reading initial positions...")
    
    # Send zero command just to read feedback
    for i, device_id in enumerate(JOINT_ORDER):
        bus = get_bus(device_id)
        pos, _ = bus.write_read_pdo_2(device_id, 0.0, 0.0) # Target 0 won't be applied instantly due to mode switch delay/feed
        
        if pos is None:
            print(f"[WARNING] Failed to read ID {device_id}. Assuming 0.0")
            pos = 0.0
        current_pos[i] = pos
        
    return current_pos

def load_trajectory(filename):
    """Load CSV and convert to numpy array."""
    try:
        df = pd.read_csv(filename)
        if 'Frame_Index' in df.columns:
            df = df.drop(columns=['Frame_Index'])
        traj = df.to_numpy()
        print(f"Loaded trajectory: {traj.shape} (Rows x Joints)")
        return traj
    except FileNotFoundError:
        print(f"[ERROR] File not found: {filename}")
        return None

# ============================================
# 4. Main Execution
# ============================================
def main():
    rate = RateLimiter(frequency=FPS)
    
    # 1. Setup
    configure_motors()
    traj_data = load_trajectory(CSV_FILE)
    if traj_data is None:
        return

    # 2. Get Start State
    # Note: We read the actual current motor positions, NOT assuming 0.0
    start_positions = read_current_positions()
    target_start_positions = traj_data[0] # The first frame of the CSV

    print(f"Current Joint Pos (First 3): {start_positions[:3]}")
    print(f"Target Start Pos (First 3): {target_start_positions[:3]}")

    try:
        # ---------------------------------------------------------
        # PHASE 1: Smooth Initialization (Current -> CSV Start)
        # ---------------------------------------------------------
        print(f"\n[Phase 1] Moving to start position over {INIT_DURATION}s...")
        
        init_steps = int(INIT_DURATION * FPS)
        
        for step in range(init_steps):
            alpha = step / float(init_steps) # 0.0 to 1.0
            
            # Interpolation: (1-a)*Current + a*Target
            current_cmd = (1 - alpha) * start_positions + alpha * target_start_positions
            
            # Send commands
            for i, device_id in enumerate(JOINT_ORDER):
                bus = get_bus(device_id)
                # Send Position Command
                bus.write_read_pdo_2(device_id, current_cmd[i], 0.0) # Pos, Vel(0)
            
            rate.sleep()

        print("[Phase 1] Complete. Ready for trajectory.")
        time.sleep(0.5) # Short pause

        # ---------------------------------------------------------
        # PHASE 2: Execute CSV Trajectory
        # ---------------------------------------------------------
        print(f"\n[Phase 2] Executing Trajectory ({len(traj_data)} frames)...")
        
        start_time = time.time()
        
        for frame_idx, joint_targets in enumerate(traj_data):
            # joint_targets is a row from CSV (numpy array)
            
            for i, device_id in enumerate(JOINT_ORDER):
                bus = get_bus(device_id)
                # Send Position Command
                # joint_targets[i] maps to JOINT_ORDER[i]
                bus.write_read_pdo_2(device_id, joint_targets[i], 0.0)
            
            # Optional: Debug Print every 50 frames (1 sec)
            if frame_idx % 50 == 0:
                print(f"Frame {frame_idx}/{len(traj_data)} executed.")

            rate.sleep()

        print("Trajectory Finished.")

    except KeyboardInterrupt:
        print("\n[Interrupted] Stopping by user request.")

    except Exception as e:
        print(f"\n[Error] {e}")

    finally:
        # ---------------------------------------------------------
        # Safety Shutdown
        # ---------------------------------------------------------
        print("\nStopping Robot (Setting IDLE mode)...")
        for device_id in JOINT_ORDER:
            bus = get_bus(device_id)
            bus.set_mode(device_id, recoil.Mode.IDLE)
            bus.write_torque_limit(device_id, 0.0) # Optional safety
        
        bus_left.stop()
        bus_right.stop()
        print("CAN Buses closed.")

if __name__ == "__main__":
    main()