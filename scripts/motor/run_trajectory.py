# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import numpy as np
import pandas as pd
import time
from cc.udp import UDP
from berkeley_humanoid_lite_lowlevel.robot import Humanoid

# --- CONFIGURATION ---
CSV_FILE = "20251212_joints_timeseries_1.csv"  # The file we created in the previous step
FPS = 50                            # Target frequency
DT = 1.0 / FPS                      # 0.02 seconds per frame
INIT_DURATION = 3.0                 # Seconds to smoothly move from 0 to Start Position

def main():
    robot = Humanoid()
    udp = UDP(("0.0.0.0", 11000), ("172.28.0.5", 11000))

    # Load the CSV Data
    try:
        df = pd.read_csv(CSV_FILE)
        if 'Frame_Index' in df.columns:
            df = df.drop(columns=['Frame_Index'])
        
        # Convert to numpy array for faster access
        joint_trajectory = df.to_numpy()
        print(f"Loaded {len(joint_trajectory)} frames from {CSV_FILE}")
        
    except FileNotFoundError:
        print(f"Error: Could not find {CSV_FILE}")
        return

    try:
        # PHASE 1: SMOOTH INITIALIZATION
        # Move from [0,0...] to the first row of the CSV
        print("Starting Smooth Initialization (0 -> Start)...")
        
        start_pos = np.zeros(12)
        target_pos = joint_trajectory[0] # The first frame of your animation
        
        # Calculate how many steps to take for initialization
        init_steps = int(INIT_DURATION * FPS)
        
        for i in range(init_steps):
            loop_start = time.time()
            
            # Linear Interpolation (Lerp)
            alpha = i / float(init_steps)
            # cmd = (1 - alpha) * start + alpha * target
            acs = (1 - alpha) * start_pos + alpha * target_pos
            
            # Send to robot
            obs = robot.step(acs)
            udp.send_numpy(obs)
            
            # Print status occasionally
            if i % 10 == 0:
                print(f"Init: {int(alpha*100)}% complete")

            # Maintain 50 FPS
            elapsed = time.time() - loop_start
            if elapsed < DT:
                time.sleep(DT - elapsed)

        print("Initialization Complete. Starting Motion...")

        # PHASE 2: PLAYBACK CSV TRAJECTORY
        for frame_idx, acs in enumerate(joint_trajectory):
            loop_start = time.time()
            
            # 'acs' is already the row from the CSV (numpy array of 12 floats)
            obs = robot.step(acs)
            udp.send_numpy(obs)

            # Optional: Print current status
            # print(f"Frame {frame_idx}/{len(joint_trajectory)}: {robot.joint_position_measured}")

            # Maintain 50 FPS
            elapsed = time.time() - loop_start
            if elapsed < DT:
                time.sleep(DT - elapsed)

        # Optional: Hold the last position for a moment before stopping?
        print("Trajectory finished.")
        
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    
    finally:
        robot.stop()
        print("Robot Stopped.")

if __name__ == "__main__":
    main()