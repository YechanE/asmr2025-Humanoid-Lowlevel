# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time
import numpy as np
from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil

# ============================================
# [USER CONTROL] 사용자 조정 변수
# ============================================
# 1. 동작 크기 배율 (0.0 ~ 1.0 권장)
# - 0.0: 제자리 서기 (Offset 자세 유지)
# - 0.5: 절반 크기로 살살 걷기
# - 1.0: CSV 원본 크기로 걷기
MOTION_SCALE = 5  

# 2. 보행 속도 (Hz)
# - CSV 분석 결과는 1.02Hz 였으나, 조금 느리게(0.8) 하거나 빠르게(1.2) 조정 가능
WALK_FREQ = 1  

# 3. 안전 시작 시간 (초)
# - 0에서 목표 진폭까지 서서히 도달하는 시간
RAMP_UP_DURATION = 4.0

# ============================================
# 1. 제어 파라미터 & 하드웨어 설정
# ============================================
FPS = 10.0   # 부드러운 제어를 위해 100Hz 이상 권장
KP = 0.2     # 위치 제어 비례 게인
KD = 0.05      # 위치 제어 미분 게인 (진동 억제)
TORQUE_LIMIT = 0.4 

bus_left = recoil.Bus(channel="can0", bitrate=1000000)
bus_right = recoil.Bus(channel="can1", bitrate=1000000)

# Joint IDs
L_HIP_ROLL, L_HIP_YAW, L_HIP_PITCH = 1, 3, 5
L_KNEE = 7
L_ANK_PITCH, L_ANK_ROLL = 11, 13

R_HIP_ROLL, R_HIP_YAW, R_HIP_PITCH = 2, 4, 6
R_KNEE = 8
R_ANK_PITCH, R_ANK_ROLL = 12, 14

LEFT_BUS_IDS = [L_HIP_ROLL, L_HIP_YAW, L_HIP_PITCH, L_KNEE, L_ANK_PITCH, L_ANK_ROLL]
ALL_IDS = [1, 2, 3, 4, 5, 6, 7, 8, 11, 12, 13, 14]

def get_bus(device_id):
    if device_id in LEFT_BUS_IDS:
        return bus_left
    return bus_right

# ============================================
# 2. 분석된 모션 파라미터 (Fitted Parameters)
# ============================================
# CSV 분석을 통해 추출한 "기본 자세(Offset)", "진폭(Amp)", "위상(Phase)"
# Target = Offset + (Amp * MOTION_SCALE * sin(wt + Phase))

# (1) Center Offsets (기본 자세, rad)
OFFSETS = {
    L_HIP_ROLL:  -0.000, L_HIP_YAW:   -0.000, L_HIP_PITCH: -0.431,
    L_KNEE:       1.093, L_ANK_PITCH: -0.565, L_ANK_ROLL:   0.000,
    
    R_HIP_ROLL:  -0.003, R_HIP_YAW:   -0.003, R_HIP_PITCH: -0.429,
    R_KNEE:       1.090, R_ANK_PITCH: -0.563, R_ANK_ROLL:   0.004
}

# (2) Base Amplitudes (원본 진폭, rad)
BASE_AMPLITUDES = {
    L_HIP_ROLL:  0.017, L_HIP_YAW:   0.021, L_HIP_PITCH: 0.026,
    L_KNEE:      0.052, L_ANK_PITCH: 0.026, L_ANK_ROLL:  0.027,
    
    R_HIP_ROLL:  0.017, R_HIP_YAW:   0.021, R_HIP_PITCH: 0.026,
    R_KNEE:      0.051, R_ANK_PITCH: 0.025, R_ANK_ROLL:  0.027
}

# (3) Phases (위상, rad)
PHASES = {
    L_HIP_ROLL:  1.54, L_HIP_YAW:   1.54, L_HIP_PITCH: -1.45,
    L_KNEE:      1.67, L_ANK_PITCH: -1.47, L_ANK_ROLL:  -1.60,
    
    R_HIP_ROLL:  1.50, R_HIP_YAW:   1.50, R_HIP_PITCH: 0.90,
    R_KNEE:      -2.18, R_ANK_PITCH: 1.01, R_ANK_ROLL:  -1.64
}

# ============================================
# 3. 헬퍼 함수
# ============================================
def configure_motors():
    print("Configuring motors...")
    for device_id in ALL_IDS:
        bus = get_bus(device_id)
        bus.write_position_kp(device_id, KP)
        bus.write_position_kd(device_id, KD)
        bus.write_torque_limit(device_id, TORQUE_LIMIT)
        bus.set_mode(device_id, recoil.Mode.POSITION)
        bus.feed(device_id)

def get_current_positions():
    pos_map = {}
    for device_id in ALL_IDS:
        bus = get_bus(device_id)
        p, _ = bus.write_read_pdo_2(device_id, 0.0, 0.0)
        pos_map[device_id] = p if p is not None else 0.0
    return pos_map

# ============================================
# 4. 메인 제어 루프
# ============================================
def main():
    rate = RateLimiter(frequency=FPS)
    configure_motors()

    # 1. 초기 위치 읽기 (Soft Start를 위한 기준)
    start_positions = get_current_positions()
    print(f"Current Pos (L_Knee): {start_positions.get(L_KNEE, 0.0):.3f}")
    
    print("="*50)
    print(f"Starting Sine Wave Motion")
    print(f" - Scale: {MOTION_SCALE * 100}%")
    print(f" - Speed: {WALK_FREQ} Hz")
    print("Press Ctrl+C to Stop.")
    print("="*50)
    
    start_time = time.time()

    try:
        while True:
            # 시간 계산
            t_now = time.time() - start_time
            
            # (1) Ramp Factor 계산 (0.0 -> 1.0)
            # 시작 시 급발진 방지: 0초~RAMP_UP_DURATION초 동안 서서히 진폭 증가
            ramp = min(t_now / RAMP_UP_DURATION, 1.0)
            
            # (2) Sine Wave 공통 위상 (omega * t)
            phase_base = 2 * np.pi * WALK_FREQ * t_now

            for device_id in ALL_IDS:
                # 파라미터 가져오기
                offset = OFFSETS.get(device_id, 0.0)
                base_amp = BASE_AMPLITUDES.get(device_id, 0.0)
                phi = PHASES.get(device_id, 0.0)
                
                # --- [핵심] 자세 보간 로직 ---
                # 목표: "현재 실제 위치"에서 시작하여 서서히 "사인파 궤적"으로 합류
                
                # A. 순수 사인파 목표값 (Pure Sine Target)
                #    Target = Center + (Amp * Scale * sin(wt + phi))
                sine_component = base_amp * MOTION_SCALE * np.sin(phase_base + phi)
                pure_target = offset + sine_component
                
                # B. 초기 위치 블렌딩 (Initial Position Blending)
                #    시작 순간(ramp=0)에는 'start_positions'를 유지하다가
                #    시간이 지날수록(ramp=1) 'pure_target'으로 전환
                init_pos = start_positions.get(device_id, 0.0)
                
                # Final Command = (1 - ramp) * Init_Pos  +  (ramp) * Pure_Target
                cmd_pos = ((1.0 - ramp) * init_pos) + (ramp * pure_target)

                # 명령 전송
                bus = get_bus(device_id)
                bus.write_read_pdo_2(device_id, cmd_pos, 0.0)
            
            rate.sleep()

    except KeyboardInterrupt:
        print("\nStopping...")

    except Exception as e:
        print(f"\nError: {e}")

    finally:
        print("Setting IDLE mode.")
        for device_id in ALL_IDS:
            bus = get_bus(device_id)
            bus.set_mode(device_id, recoil.Mode.IDLE)
        
        bus_left.stop()
        bus_right.stop()

if __name__ == "__main__":
    main()