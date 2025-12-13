    # Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

    import time
    import numpy as np
    import pandas as pd
    from loop_rate_limiters import RateLimiter
    import berkeley_humanoid_lite_lowlevel.recoil as recoil

    # ============================================
    # 1. 설정 (Configuration)
    # ============================================
    CSV_FILE = "20251212_joints_timeseries_1.csv"
    FPS = 50.0
    INIT_DURATION = 2.0                 # 오프셋 맞춤 후 대기/준비 시간

    # 게인 설정 (상대 제어이므로 궤적 추종성 중요)
    KP = 4.0  
    KD = 0.1
    TORQUE_LIMIT = 3.0

    # ============================================
    # 2. 하드웨어 매핑 (Hardware Mapping)
    # ============================================
    bus_left = recoil.Bus(channel="can0", bitrate=1000000)
    bus_right = recoil.Bus(channel="can1", bitrate=1000000)

    L_HIP_ROLL, L_HIP_YAW, L_HIP_PITCH = 1, 3, 5
    L_KNEE = 7
    L_ANK_PITCH, L_ANK_ROLL = 11, 13

    R_HIP_ROLL, R_HIP_YAW, R_HIP_PITCH = 2, 4, 6
    R_KNEE = 8
    R_ANK_PITCH, R_ANK_ROLL = 12, 14

    LEFT_BUS_IDS = [L_HIP_ROLL, L_HIP_YAW, L_HIP_PITCH, L_KNEE, L_ANK_PITCH, L_ANK_ROLL]

    # CSV 컬럼 순서와 1:1 매칭되는 하드웨어 ID 리스트
    JOINT_ORDER = [
        L_ANK_ROLL, L_ANK_PITCH, L_KNEE, L_HIP_PITCH, L_HIP_YAW, L_HIP_ROLL,
        R_ANK_ROLL, R_ANK_PITCH, R_KNEE, R_HIP_PITCH, R_HIP_YAW, R_HIP_ROLL
    ]

    def get_bus(device_id):
        if device_id in LEFT_BUS_IDS:
            return bus_left
        return bus_right

    # ============================================
    # 3. 헬퍼 함수
    # ============================================
    def configure_motors():
        print("Configuring motors...")
        for device_id in set(JOINT_ORDER):
            bus = get_bus(device_id)
            bus.write_position_kp(device_id, KP)
            bus.write_position_kd(device_id, KD)
            bus.write_torque_limit(device_id, TORQUE_LIMIT)
            bus.set_mode(device_id, recoil.Mode.POSITION)
            bus.feed(device_id)

    def load_trajectory(filename):
        try:
            df = pd.read_csv(filename)
            cols_to_drop = [c for c in df.columns if 'Index' in c or 'Time' in c]
            if cols_to_drop:
                df = df.drop(columns=cols_to_drop)
            traj = df.to_numpy()
            print(f"Loaded CSV: {traj.shape}")
            return traj
        except Exception as e:
            print(f"[ERROR] CSV Load Failed: {e}")
            return None

    # ============================================
    # 4. 메인 실행 (Relative Control Logic Applied)
    # ============================================
    def main():
        rate = RateLimiter(frequency=FPS)
        
        # 1. 설정 및 데이터 로드
        configure_motors()
        traj_data = load_trajectory(CSV_FILE)
        if traj_data is None: return

        # 2. 현재 로봇의 물리적 위치 읽기 (Zero Reference)
        print("\n[Calibration] Reading current positions as ZERO REFERENCE...")
        start_pos_actual = np.zeros(len(JOINT_ORDER))
        
        for i, device_id in enumerate(JOINT_ORDER):
            bus = get_bus(device_id)
            # 명령 없이 현재 값만 읽어옴
            pos, _ = bus.write_read_pdo_2(device_id, 0.0, 0.0)
            start_pos_actual[i] = pos if pos is not None else 0.0

        # 3. 오프셋 계산 (핵심 로직)
        # 목표: 로봇의 현재 위치(start_pos_actual)가 CSV의 시작점(traj_data[0])과 일치하도록 이동시킴
        # Offset = Actual_Current - CSV_First_Frame
        csv_start_frame = traj_data[0]
        position_offset = start_pos_actual - csv_start_frame
        
        print("-" * 50)
        print(f"Calculated Relative Offsets (First 3 joints):")
        print(f"Current: {start_pos_actual[:3]}")
        print(f"CSV Start: {csv_start_frame[:3]}")
        print(f"Offset applied: {position_offset[:3]}")
        print("-" * 50)
        
        print("Ready to start relative motion. Press Ctrl+C to stop.")
        time.sleep(1.0)

        try:
            # ---------------------------------------------------------
            # PHASE 1: Alignment (정렬)
            # 이미 오프셋을 계산했으므로, 첫 명령은 정확히 현재 위치가 됩니다.
            # 혹시 모를 미세 진동을 잡기 위해 1초간 홀딩합니다.
            # ---------------------------------------------------------
            print(f"\n[Phase 1] Holding start position for {INIT_DURATION}s...")
            hold_steps = int(INIT_DURATION * FPS)
            
            # 첫 번째 타겟(CSV[0]) + 오프셋 = 현재 위치
            hold_target = csv_start_frame + position_offset 
            
            for _ in range(hold_steps):
                for i, device_id in enumerate(JOINT_ORDER):
                    bus = get_bus(device_id)
                    bus.write_read_pdo_2(device_id, hold_target[i], 0.0)
                rate.sleep()

            print("[Phase 1] Alignment Complete. Starting Trajectory...")

            # ---------------------------------------------------------
            # PHASE 2: Relative Trajectory Playback
            # ---------------------------------------------------------
            print(f"\n[Phase 2] Playing Trajectory relative to start pose...")
            
            for frame_idx, csv_row in enumerate(traj_data):
                # csv_row: CSV의 현재 프레임 데이터
                
                # [Relative Target Calculation]
                # Target = CSV_Data + Offset
                target_cmds = csv_row + position_offset
                
                for i, device_id in enumerate(JOINT_ORDER):
                    bus = get_bus(device_id)
                    # 계산된 상대 좌표 전송
                    bus.write_read_pdo_2(device_id, target_cmds[i], 0.0)
                
                if frame_idx % 50 == 0:
                    print(f"Frame {frame_idx}/{len(traj_data)}")

                rate.sleep()

            print("Trajectory Finished.")

        except KeyboardInterrupt:
            print("\nStopping by user...")

        except Exception as e:
            print(f"\nRuntime Error: {e}")

        finally:
            print("Setting IDLE mode.")
            for device_id in set(JOINT_ORDER):
                bus = get_bus(device_id)
                bus.set_mode(device_id, recoil.Mode.IDLE)
            
            bus_left.stop()
            bus_right.stop()

    if __name__ == "__main__":
        main()