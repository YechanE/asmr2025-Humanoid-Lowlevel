# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time
import numpy as np

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil

# argparse
args = recoil.util.get_args()

# CAN 통신
bus = recoil.Bus(channel=args.channel, bitrate=1000000)

# 읽을 모터 ID 리스트
device_ids = [1, 3, 5, 7, 11, 13]

rate = RateLimiter(frequency=100.0)   # 100 Hz로 읽기

print("Reading encoder positions (no motion)...")

try:
    while True:
        for device_id in device_ids:

            # 모터에 명령(토크=0, 목표값=0) 보내면서 현재 pos/vel 받기
            pos, vel = bus.write_read_pdo_2(device_id, 0.0, 0.0)

            if pos is not None and vel is not None:
                print(
                    f"ID {device_id:2d} | "
                    f"pos: {pos:.3f} rad   vel: {vel:.3f} rad/s"
                )
            else:
                print(f"ID {device_id:2d} | Read failed")

        rate.sleep()

except KeyboardInterrupt:
    pass

print("Stopping CAN bus...")
bus.stop()
