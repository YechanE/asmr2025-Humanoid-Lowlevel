# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time
import numpy as np

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil


args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1_000_000)
device_id = args.id

# --- Gains / limits ---
kp = 0.2
kd = 0.01
bus.write_position_kp(device_id, kp)
bus.write_position_kd(device_id, kd)
bus.write_torque_limit(device_id, 1)

bus.set_mode(device_id, recoil.Mode.POSITION)
bus.feed(device_id)

time.sleep(0.01)

print("Reading initial position...")
initial_position, _ = bus.write_read_pdo_2(device_id, 0.0, 0.0)
if initial_position is None:
    print("Failed to read initial position. Using 0.0 rad.")
    initial_position = 0.0

print(f"Initial Position = {initial_position:.3f} rad")

# ============================================================
# Move profile settings (YOU CHANGE THESE)
# ============================================================
DELTA_RAD = 2      # initial_position에서 +0.5 rad 만큼 이동 (원하면 음수도 가능)
MOVE_TIME = 2.0       # 2초에 걸쳐 서서히 이동 (램프 시간)
HOLD_TIME = 1.0       # 목표 위치에서 1초 유지 후 종료 (원하면 None으로 무한 유지)

CONTROL_HZ = 200.0
rate = RateLimiter(frequency=CONTROL_HZ)

start_pos = initial_position
goal_pos = start_pos + DELTA_RAD

t0 = time.time()
t_move_end = t0 + MOVE_TIME

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

try:
    # ----------------------------
    # 1) Ramp move: start_pos -> goal_pos
    # ----------------------------
    while True:
        now = time.time()
        if now >= t_move_end:
            break

        alpha = (now - t0) / MOVE_TIME          # 0 -> 1
        alpha = clamp(alpha, 0.0, 1.0)
        target_angle = (1.0 - alpha) * start_pos + alpha * goal_pos

        measured_position, measured_velocity = bus.write_read_pdo_2(device_id, target_angle, 0.0)
        if measured_position is not None and measured_velocity is not None:
            print(f"Measured pos: {measured_position:.3f}\tvel: {measured_velocity:.3f}\ttarget: {target_angle:.3f}")

        rate.sleep()

    # ----------------------------
    # 2) Hold at goal_pos (stop moving)
    # ----------------------------
    print(f"Reached goal. Holding at {goal_pos:.3f} rad")

    if HOLD_TIME is None:
        # 무한 유지
        while True:
            measured_position, measured_velocity = bus.write_read_pdo_2(device_id, goal_pos, 0.0)
            if measured_position is not None and measured_velocity is not None:
                print(f"Measured pos: {measured_position:.3f}\tvel: {measured_velocity:.3f}\ttarget: {goal_pos:.3f}")
            rate.sleep()
    else:
        hold_end = time.time() + HOLD_TIME
        while time.time() < hold_end:
            measured_position, measured_velocity = bus.write_read_pdo_2(device_id, goal_pos, 0.0)
            if measured_position is not None and measured_velocity is not None:
                print(f"Measured pos: {measured_position:.3f}\tvel: {measured_velocity:.3f}\ttarget: {goal_pos:.3f}")
            rate.sleep()

except KeyboardInterrupt:
    pass

bus.set_mode(device_id, recoil.Mode.IDLE)
bus.stop()
