# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time
import csv
import numpy as np

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil


# ============================================================
# 0. User Config
# ============================================================

CSV_PATH = "20251212_joints_timeseries_1.csv"

CONTROL_FREQUENCY = 60.0  # Hz
rate = RateLimiter(frequency=CONTROL_FREQUENCY)

GO_TO_FIRST_FRAME_DURATION = 2.0  # sec

KP_POSITION = 0.2
KD_POSITION = 0.01
TORQUE_LIMIT = 0.4

CSV_IS_ABSOLUTE_ANGLE = True  # CSV는 절대각(rad)


# ============================================================
# 1. CAN bus / Motor IDs
# ============================================================

bus_left  = recoil.Bus(channel="can0", bitrate=1_000_000)
bus_right = recoil.Bus(channel="can1", bitrate=1_000_000)

LEFT_ANKLE_ROLL, LEFT_ANKLE_PITCH = 13, 11
LEFT_KNEE_PITCH = 7
LEFT_HIP_PITCH, LEFT_HIP_YAW, LEFT_HIP_ROLL = 5, 3, 1

RIGHT_ANKLE_ROLL, RIGHT_ANKLE_PITCH = 14, 12
RIGHT_KNEE_PITCH = 8
RIGHT_HIP_PITCH, RIGHT_HIP_YAW, RIGHT_HIP_ROLL = 6, 4, 2

LEFT_IDS = {
    LEFT_ANKLE_ROLL, LEFT_ANKLE_PITCH,
    LEFT_KNEE_PITCH,
    LEFT_HIP_PITCH, LEFT_HIP_YAW, LEFT_HIP_ROLL
}

def get_bus(device_id: int):
    return bus_left if device_id in LEFT_IDS else bus_right


# ============================================================
# 1.5 Direction map (device_id -> +1/-1)
#     여기만 바꾸면 전체 방향이 바뀜
# ============================================================

DIRECTION = {
    LEFT_HIP_ROLL:  -1, LEFT_HIP_YAW:  1, LEFT_HIP_PITCH:  1,
    LEFT_KNEE_PITCH: 1, LEFT_ANKLE_PITCH: 1, LEFT_ANKLE_ROLL:  -1,

    RIGHT_HIP_ROLL: 1, RIGHT_HIP_YAW: -1, RIGHT_HIP_PITCH: -1,
    RIGHT_KNEE_PITCH:  -1, RIGHT_ANKLE_PITCH:  -1, RIGHT_ANKLE_ROLL: -1,
}

def dir_of(did: int) -> float:
    # 매핑 누락 시 즉시 에러 내서 “조용히 잘못 움직이는” 상황 방지
    if did not in DIRECTION:
        raise KeyError(f"DIRECTION missing for device_id={did}")
    return float(DIRECTION[did])


# ============================================================
# 2. Joint name ↔ Motor ID (CSV 순서 그대로)
# ============================================================

JOINT_ORDER = [
    "leg_left_ankle_roll_joint",
    "leg_left_ankle_pitch_joint",
    "leg_left_knee_pitch_joint",
    "leg_left_hip_pitch_joint",
    "leg_left_hip_yaw_joint",
    "leg_left_hip_roll_joint",

    "leg_right_ankle_roll_joint",
    "leg_right_ankle_pitch_joint",
    "leg_right_knee_pitch_joint",
    "leg_right_hip_pitch_joint",
    "leg_right_hip_yaw_joint",
    "leg_right_hip_roll_joint",
]

JOINT_ID = {
    "leg_left_ankle_roll_joint":  LEFT_ANKLE_ROLL,
    "leg_left_ankle_pitch_joint": LEFT_ANKLE_PITCH,
    "leg_left_knee_pitch_joint":  LEFT_KNEE_PITCH,
    "leg_left_hip_pitch_joint":   LEFT_HIP_PITCH,
    "leg_left_hip_yaw_joint":     LEFT_HIP_YAW,
    "leg_left_hip_roll_joint":    LEFT_HIP_ROLL,

    "leg_right_ankle_roll_joint":  RIGHT_ANKLE_ROLL,
    "leg_right_ankle_pitch_joint": RIGHT_ANKLE_PITCH,
    "leg_right_knee_pitch_joint":  RIGHT_KNEE_PITCH,
    "leg_right_hip_pitch_joint":   RIGHT_HIP_PITCH,
    "leg_right_hip_yaw_joint":     RIGHT_HIP_YAW,
    "leg_right_hip_roll_joint":    RIGHT_HIP_ROLL,
}

ALL_IDS = list(JOINT_ID.values())


# ============================================================
# 3. CSV Loading (Frame_Index 있으면 무시, joint 컬럼만 사용)
# ============================================================

def load_csv(csv_path: str):
    traj = {name: [] for name in JOINT_ORDER}

    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise RuntimeError("CSV has no header row")

        # 필요한 컬럼 체크 (Frame_Index는 있어도 되고 없어도 됨)
        missing = [name for name in JOINT_ORDER if name not in reader.fieldnames]
        if missing:
            raise KeyError(f"CSV missing columns: {missing}")

        for row_idx, row in enumerate(reader):
            for name in JOINT_ORDER:
                try:
                    traj[name].append(float(row[name]))
                except Exception as e:
                    raise ValueError(f"Bad value at row={row_idx}, col='{name}': {row.get(name)}") from e

    T = len(next(iter(traj.values()))) if traj else 0
    if T <= 0:
        raise RuntimeError("CSV trajectory is empty")

    traj = {k: np.asarray(v, dtype=np.float64) for k, v in traj.items()}
    return traj, T


# ============================================================
# 4. Motor helpers
# ============================================================

def configure_motor(device_id: int):
    bus = get_bus(device_id)
    bus.write_position_kp(device_id, KP_POSITION)
    bus.write_position_kd(device_id, KD_POSITION)
    bus.write_torque_limit(device_id, TORQUE_LIMIT)
    bus.set_mode(device_id, recoil.Mode.POSITION)
    bus.feed(device_id)

def read_position(device_id: int):
    bus = get_bus(device_id)
    pos, vel = bus.write_read_pdo_2(device_id, 0.0, 0.0)
    return float(pos or 0.0), float(vel or 0.0)


# ============================================================
# 4.5 CSV(joint angle) -> motor command 변환 (여기서 DIRECTION 적용)
# ============================================================

def csv_to_motor_cmd(joint_name: str, csv_value: float, init_pos_map: dict):
    """
    csv_value: CSV의 joint angle(rad) (절대각 or delta)
    return: 모터에 보낼 position command(rad)
    """
    did = JOINT_ID[joint_name]
    sign = dir_of(did)

    if CSV_IS_ABSOLUTE_ANGLE:
        joint_target = csv_value
    else:
        joint_target = init_pos_map[joint_name] + csv_value  # init_pos가 joint 좌표라고 가정할 때만 의미 있음

    # 방향 반전 적용: 모터가 보는 좌표계로 변환
    return sign * float(joint_target)


# ============================================================
# 5. Main
# ============================================================

def main():
    traj, T = load_csv(CSV_PATH)
    print(f"[INFO] CSV loaded: {T} frames from '{CSV_PATH}'")

    print("[INFO] Configuring motors...")
    for did in ALL_IDS:
        configure_motor(did)
    time.sleep(0.2)

    print("[INFO] Reading initial positions (motor coordinates)...")
    init_pos = {}
    for name in JOINT_ORDER:
        did = JOINT_ID[name]
        pos, _ = read_position(did)
        init_pos[name] = pos
        print(f"  {name:30s} ID {did:2d}  init={pos:+.4f}  dir={int(dir_of(did)):+d}")

    # --------------------------------------------------------
    # Phase A: move to first CSV frame (motor command space)
    # --------------------------------------------------------
    print("\n[INFO] Phase A: Go to first CSV frame (with DIRECTION)")
    target0_motor = {}
    for name in JOINT_ORDER:
        target0_motor[name] = csv_to_motor_cmd(name, float(traj[name][0]), init_pos)

    t0 = time.time()
    while True:
        s = min((time.time() - t0) / GO_TO_FIRST_FRAME_DURATION, 1.0)
        s = s * s * (3.0 - 2.0 * s)  # smoothstep

        for name in JOINT_ORDER:
            did = JOINT_ID[name]
            bus = get_bus(did)

            cmd = (1.0 - s) * init_pos[name] + s * target0_motor[name]
            bus.write_read_pdo_2(did, cmd, 0.0)

        if s >= 1.0:
            break
        rate.sleep()

    time.sleep(0.3)
    print("[INFO] Phase A done.\n")

    # --------------------------------------------------------
    # Phase B: CSV playback (motor command space)
    # --------------------------------------------------------
    print("[INFO] Phase B: CSV playback (with DIRECTION)")
    idx = 0

    try:
        while True:
            for name in JOINT_ORDER:
                did = JOINT_ID[name]
                bus = get_bus(did)

                csv_v = float(traj[name][idx]) if idx < T else float(traj[name][-1])
                cmd = csv_to_motor_cmd(name, csv_v, init_pos)

                bus.write_read_pdo_2(did, cmd, 0.0)

            if idx % int(CONTROL_FREQUENCY) == 0:
                print(f"[INFO] frame {idx}/{T}")

            idx += 1
            rate.sleep()

    except KeyboardInterrupt:
        print("\n[INFO] Stopping...")

    finally:
        print("[INFO] Set all motors to IDLE")
        for did in ALL_IDS:
            get_bus(did).set_mode(did, recoil.Mode.IDLE)
        bus_left.stop()
        bus_right.stop()
        print("[INFO] Buses stopped. Program terminated.")


if __name__ == "__main__":
    main()
