# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time
import numpy as np

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil


args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)

device_id = args.id

# 더 강한 게인 설정
kp = 1.0          # 위치 게인 증가
kd = 0.05         # 속도 감쇠 증가
torque_limit = 0.5  # 토크 제한 증가

rate = RateLimiter(frequency=200.0)

bus.write_position_kp(device_id, kp)
bus.write_position_kd(device_id, kd)
bus.write_torque_limit(device_id, torque_limit)

bus.set_mode(device_id, recoil.Mode.POSITION)
bus.feed(device_id)

time.sleep(0.2)  # 모드 전환 대기

# 루프 내에서 초기 위치 읽기
measured_position, measured_velocity = bus.write_read_pdo_2(device_id, 0.0, 0.0)
if measured_position is None:
    print("Failed to read initial position")
    bus.stop()
    exit(1)

initial_position = measured_position
print(f"Starting position: {initial_position:.3f} rad")

# 목표: 한 바퀴 (2π rad)
target_rotation = 2 * np.pi
duration = 5.0  # 5초 동안 한 바퀴

start_time = time.time()
last_print_time = start_time

try:
    while True:
        elapsed_time = time.time() - start_time
        
        # 부드러운 궤적 생성 (S-curve)
        if elapsed_time < duration:
            # 0 -> 1로 부드럽게 증가
            t = elapsed_time / duration
            # S-curve를 위한 smoothstep 함수
            smooth_t = t * t * (3.0 - 2.0 * t)
            target_angle = initial_position + target_rotation * smooth_t
        else:
            # 정확히 한 바퀴
            target_angle = initial_position + target_rotation
        
        measured_position, measured_velocity = bus.write_read_pdo_2(device_id, target_angle, 0.0)
        
        # 1초에 한 번씩 출력 (너무 많은 출력 방지)
        if measured_position is not None and (time.time() - last_print_time) > 0.2:
            current_rotation = measured_position - initial_position
            percent = (current_rotation / target_rotation) * 100
            print(f"Progress: {current_rotation:.3f}/{target_rotation:.3f} rad ({percent:.1f}%) \tVel: {measured_velocity:.3f}")
            last_print_time = time.time()
        
        # 회전 완료 확인
        if elapsed_time >= duration + 1.0:  # 1초 추가 대기
            if measured_position is not None:
                final_rotation = measured_position - initial_position
                print(f"\nRotation complete! Final: {final_rotation:.3f} rad ({(final_rotation/target_rotation)*100:.1f}%)")
            break
        
        rate.sleep()

except KeyboardInterrupt:
    print("\nInterrupted by user")

bus.set_mode(device_id, recoil.Mode.IDLE)
bus.stop()