# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time
import numpy as np

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil


args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)

# 동시에 구동할 CAN ID 목록
device_ids = [5,7,11]

# gains
kp = 0.6
kd = 0.01

frequency = 1.0   # motion frequency is 1 Hz
amplitude = 1.5   # motion amplitude is 1 rad

rate = RateLimiter(frequency=200.0)
# 각 모터별 제어 파라미터/모드 설정
for device_id in device_ids:
    bus.write_position_kp(device_id, kp)
    bus.write_position_kd(device_id, kd)
    bus.write_torque_limit(device_id, 1.5)

    bus.set_mode(device_id, recoil.Mode.POSITION)
    bus.feed(device_id)

# 각 모터별 초기 위치 읽기
initial_positions = {}

print("Reading initial positions...")
for device_id in device_ids:
    initial_position, _ = bus.write_read_pdo_2(device_id, 0.0, 0.0)
    if initial_position is None:
        print(f"Failed to read initial position for ID {device_id}. Use 0.0 rad.")
        initial_position = 0.0

    initial_positions[device_id] = initial_position
    print(f"ID {device_id}: Initial Position = {initial_position:.3f} rad")

print("Start sinusoidal motion around each initial position...")

try:
    while True:
        # 시간 기준 공통 사인파 (phase/진폭 동일)
        base_angle = np.sin(2 * np.pi * frequency * time.time()) * amplitude

        for device_id in device_ids:
            # 🔹 각 모터의 '자기 초기 위치' 기준으로 좌우로 움직이도록
            initial_position = initial_positions[device_id]
            target_angle = initial_position + base_angle

            measured_position, measured_velocity = bus.write_read_pdo_2(
                device_id, target_angle, 0.0
            )

            if measured_position is not None and measured_velocity is not None:
                print(
                    f"ID {device_id:2d} | "
                    f"target: {target_angle:.3f} rad | "
                    f"pos: {measured_position:.3f} rad | "
                    f"vel: {measured_velocity:.3f} rad/s"
                )

        rate.sleep()

except KeyboardInterrupt:
    pass

# 모든 모터를 IDLE로 전환
for device_id in device_ids:
    bus.set_mode(device_id, recoil.Mode.IDLE)

bus.stop()
