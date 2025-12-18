# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time
import csv

import numpy as np
from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil


# ============================================================
# 1. Argument & Bus Initialization
# ============================================================

args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1_000_000)
device_id = args.id


# ============================================================
# 2. Control Parameters
# ============================================================

KP_POSITION = 0.2
KD_POSITION = 0.01
TORQUE_LIMIT = 0.4

CONTROL_FREQUENCY = 50.0  # Hz
rate = RateLimiter(frequency=CONTROL_FREQUENCY)



# ============================================================
# 3. CSV Trajectory Loading
# ============================================================

CSV_PATH = "20251212_joints_timeseries_1.csv"  # <-- 실제 경로로 수정

trajectory = []

with open(CSV_PATH, newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        trajectory.append(float(row["leg_left_knee_pitch_joint"]))

trajectory_length = len(trajectory)

if trajectory_length == 0:
    raise RuntimeError("CSV trajectory is empty")

print(f"[INFO] CSV trajectory loaded: {trajectory_length} frames")


# ============================================================
# 4. Motor Configuration
# ============================================================

print(f"[INFO] Configuring motor ID {device_id}")

bus.write_position_kp(device_id, KP_POSITION)
bus.write_position_kd(device_id, KD_POSITION)
bus.write_torque_limit(device_id, TORQUE_LIMIT)

bus.set_mode(device_id, recoil.Mode.POSITION)
bus.feed(device_id)

time.sleep(0.01)

# Initial position read (for logging only)
initial_position, _ = bus.write_read_pdo_2(device_id, 0.0, 0.0)
if initial_position is None:
    initial_position = 0.0

print(f"[INFO] Initial Position: {initial_position:.4f} rad")


# ============================================================
# 5. Main Control Loop (CSV Playback)
# ============================================================

print("\n[INFO] CSV trajectory playback started\n")

csv_index = 0
start_time = time.time()

constant = trajectory[csv_index]          # 목표로 하는 최종 상수 이동량
STEP_COUNT = 200         # 점진적 이동을 위한 총 단계 수
INCREMENT_PER_STEP = constant / STEP_COUNT # 한 단계당 증가량
current_constant_increment = 0.0 # 현재 누적된 점진적 상수 값
step_counter = 0         # 현재 스텝 카운터


try:
    while True:
        current_time = time.time() - start_time
        # --- 시퀀스 제어 로직 ---
        if step_counter < STEP_COUNT:
            # Phase 1: Ramping (점진적 증가)
            current_constant_increment += INCREMENT_PER_STEP
            step_counter += 1
            # 이 단계에서는 사인파 항은 0입니다. (Pure Ramp)
            dynamic_component = 0.0
            
            if step_counter == STEP_COUNT:
                print(f"** Phase 2: Ramping Complete. Sinusoidal Motion Start at T={current_time:.3f}s **")
                time.sleep(0.6)
        else:
            # ----------------------------------------------------
            # Target selection
            # ----------------------------------------------------
            if csv_index < trajectory_length:
                dynamic_component = trajectory[csv_index]
                csv_index += 1
            else:
                # End of CSV → hold last value
                dynamic_component = trajectory[-1]


        target_position = initial_position + current_constant_increment + dynamic_component
        measured_position, measured_velocity = bus.write_read_pdo_2(
            device_id,
            target_position,
            0.0  # no velocity feedforward
        )

        # ----------------------------------------------------
        # Logging (reduced frequency)
        # ----------------------------------------------------
        if csv_index % 40 == 0:
            print(
                f"[CSV {csv_index:04d}/{trajectory_length}] "
                f"Target: {target_position:+.4f} rad | "
                f"Measured: {measured_position:+.4f} rad"
            )

        # ----------------------------------------------------
        # Loop timing
        # ----------------------------------------------------
        rate.sleep()

except KeyboardInterrupt:
    print("\n[INFO] KeyboardInterrupt received. Stopping control loop.")


# ============================================================
# 6. Shutdown
# ============================================================

bus.set_mode(device_id, recoil.Mode.IDLE)
bus.stop()

print("[INFO] Motor set to IDLE. Program terminated.")
