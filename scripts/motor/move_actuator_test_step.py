# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time
import numpy as np

from loop_rate_limiters import RateLimiter
# berkeley_humanoid_lite_lowlevel.recoil 모듈을 recoil로 임포트
import berkeley_humanoid_lite_lowlevel.recoil as recoil


# --- 1. 변수 정의 및 초기 설정 ---
args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)

device_id = args.id

# 제어 게인 설정
kp = 0.2  
kd = 0.01

# 사인파 궤적 설정
frequency = 1.0  # 모션 주파수: 1 Hz
amplitude = 2.0  # 모션 진폭: 2.0 rad

# 제어 루프 주파수 설정
rate = RateLimiter(frequency=200.0) # 제어 주기 T_loop = 1/200 = 0.005초

# --- 2. 점진적 이동을 위한 변수 설정 ---
constant = 3.14           # 목표로 하는 최종 상수 이동량
STEP_COUNT = 100         # 점진적 이동을 위한 총 단계 수
INCREMENT_PER_STEP = constant / STEP_COUNT # 한 단계당 증가량
current_constant_increment = 0.0 # 현재 누적된 점진적 상수 값
step_counter = 0         # 현재 스텝 카운터

# --- 3. 통신 및 게인 설정 ---
print(f"Reading default values for ID {device_id}...")
pos_kp = bus.read_position_kp(device_id)
pos_kd = bus.read_position_kd(device_id)
torque_limit = bus.read_torque_limit(device_id)
print("==="*5)
print(f"Default Position KP: {pos_kp}")
print(f"Default Position KD: {pos_kd}")
print(f"Default Torque Limit: {torque_limit}")
print("==="*5)

bus.write_position_kp(device_id, kp)
bus.write_position_kd(device_id, kd)
bus.write_torque_limit(device_id, 0.4)

bus.set_mode(device_id, recoil.Mode.POSITION)
bus.feed(device_id)

time.sleep(0.001)
# 설정된 값 확인 (엔지니어링적 검증)
pos_kp = bus.read_position_kp(device_id)
pos_kd = bus.read_position_kd(device_id)
torque_limit = bus.read_torque_limit(device_id)
print("==="*5)
print(f"Set Position KP: {pos_kp}")
print(f"Set Position KD: {pos_kd}")
print(f"Set Torque Limit: {torque_limit}")
print("==="*5)
time.sleep(0.01)

print("Reading initial position...")
initial_position, _ = bus.write_read_pdo_2(device_id, 0.0, 0.0)
if initial_position is None:
    print("Failed to read initial position. Setting to 0.0.")
    initial_position = 0.0

print(f"Initial Position = {initial_position:.3f} rad")

# --- 4. 메인 제어 루프 시작 ---
start_time = time.time()
print("\n** Phase 1: Constant Ramping Start (100 Steps) **")

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
            # Phase 2: Sinusoidal Motion (사인파 중첩)
            # current_constant_increment는 constant 최종 값으로 고정됩니다.
            current_constant_increment = constant
            # 사인파 운동 시작
            dynamic_component = np.sin(2 * np.pi * frequency * current_time) * amplitude

        
        # 목표 각도 계산: 초기 위치 + 점진적 오프셋 + 동적 성분
        target_angle = initial_position + current_constant_increment + dynamic_component

        # PDO 통신
        measured_position, measured_velocity = bus.write_read_pdo_2(device_id, target_angle, 0.0)
        
        # 결과 출력 (Phase 1에서는 상세하게, Phase 2에서는 요약해서 출력 가능)
        if measured_position is not None and measured_velocity is not None:
            if step_counter <= STEP_COUNT:
                print(f"[Phase 1, Step {step_counter:03d}/{STEP_COUNT}] C_inc: {current_constant_increment:.5f} \tTarget Pos: {target_angle:.5f}")
            elif step_counter % 40 == 0 : # Phase 2에서는 40루프마다 한 번씩 출력 (출력 부하 감소)
                print(f"[Phase 2, T={current_time:.3f}s] Target Pos: {target_angle:.3f} \tMeasured Pos: {measured_position:.3f}")

        # 제어 주기 유지
        rate.sleep()



except KeyboardInterrupt:
    print("\n\n** 프로그램 종료 (KeyboardInterrupt) **")
    pass

# --- 5. 종료 작업 ---
bus.set_mode(device_id, recoil.Mode.IDLE)
bus.stop()