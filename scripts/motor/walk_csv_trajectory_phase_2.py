# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time
import csv
import numpy as np

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil


# ============================================================
# 0. User Config
# ============================================================

CSV_PATH = "joints_timeseries_fast2.csv"

CONTROL_FREQUENCY = 60.0  # Hz
rate = RateLimiter(frequency=CONTROL_FREQUENCY)

GO_TO_FIRST_FRAME_DURATION = 2.0  # sec

KP_POSITION = 0.2
KD_POSITION = 0.01
TORQUE_LIMIT = 0.4

CSV_IS_ABSOLUTE_ANGLE = True  # CSV는 절대각(rad)

# -----------------------------
# Phase C (Walking) Config
# -----------------------------
ENABLE_WALK_PHASE = True

# "loop"             : CSV가 이미 양다리 gait cycle(한 주기)일 때 그대로 반복
# "alternate_mirror" : CSV가 한 발 step만 있을 때, 다음 step은 좌/우를 바꿔서 반복
WALK_MODE = "loop"

# CSV 한 주기를 몇 초에 재생할지 (속도 조절 핵심 파라미터)
GAIT_CYCLE_SECONDS = 16.0  # 예: 1.0초에 1주기, 2.0초에 1주기(더 느림)

# loop 경계에서 튀는 걸 줄이기 위한 cross-fade (초)
LOOP_CROSSFADE_SEC = 0.08

# 걷기 시작 전에 첫 자세(0프레임)로 이동 후, 걷기 시작할 때 추가로 hold 시간
PRE_WALK_HOLD_SEC = 0.3

# -----------------------------
# Amplitude scaling (NEW)
# -----------------------------
AMPLITUDE_SCALE = 15.0  # 0.7=진폭 감소, 1.3=진폭 증가
# 절대각 CSV에서 의미 있게 진폭 조절하려면 기준 자세가 필요함:
# - "about_first_frame": 0프레임 자세를 기준(중립)으로 스윙만 확대/축소 (권장)
# - "about_zero"       : 0rad 기준으로 확대/축소
SCALE_MODE = "about_first_frame"  # "about_first_frame" or "about_zero"


# ============================================================
# 1. CAN bus / Motor IDs
# ============================================================

bus_left  = recoil.Bus(channel="can0", bitrate=1_000_000)
bus_right = recoil.Bus(channel="can1", bitrate=1_000_000)

LEFT_ANKLE_ROLL, LEFT_ANKLE_PITCH = 13, 11
LEFT_KNEE_PITCH = 7
LEFT_HIP_PITCH, LEFT_HIP_YAW, LEFT_HIP_ROLL = 5, 1, 3

RIGHT_ANKLE_ROLL, RIGHT_ANKLE_PITCH = 14, 12
RIGHT_KNEE_PITCH = 8
RIGHT_HIP_PITCH, RIGHT_HIP_YAW, RIGHT_HIP_ROLL = 6, 2, 4

LEFT_IDS = {
    LEFT_ANKLE_ROLL, LEFT_ANKLE_PITCH,
    LEFT_KNEE_PITCH,
    LEFT_HIP_PITCH, LEFT_HIP_YAW, LEFT_HIP_ROLL
}

def get_bus(device_id: int):
    return bus_left if device_id in LEFT_IDS else bus_right


# ============================================================
# 1.5 Direction map (device_id -> +1/-1)
# ============================================================

DIRECTION = {
    LEFT_HIP_ROLL:  1,  LEFT_HIP_YAW:  -1, LEFT_HIP_PITCH:  1,
    LEFT_KNEE_PITCH: 1, LEFT_ANKLE_PITCH: 1, LEFT_ANKLE_ROLL: -1,

    RIGHT_HIP_ROLL:  1, RIGHT_HIP_YAW: -1, RIGHT_HIP_PITCH: -1,
    RIGHT_KNEE_PITCH: -1, RIGHT_ANKLE_PITCH: -1, RIGHT_ANKLE_ROLL: -1,
}

def dir_of(did: int) -> float:
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
    "leg_left_ankle_roll_joint":   LEFT_ANKLE_ROLL,
    "leg_left_ankle_pitch_joint":  LEFT_ANKLE_PITCH,
    "leg_left_knee_pitch_joint":   LEFT_KNEE_PITCH,
    "leg_left_hip_pitch_joint":    LEFT_HIP_PITCH,
    "leg_left_hip_yaw_joint":      LEFT_HIP_YAW,
    "leg_left_hip_roll_joint":     LEFT_HIP_ROLL,

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

        missing = [name for name in JOINT_ORDER if name not in reader.fieldnames]
        if missing:
            raise KeyError(f"CSV missing columns: {missing}")

        for row_idx, row in enumerate(reader):
            for name in JOINT_ORDER:
                try:
                    traj[name].append(float(row[name]))
                except Exception as e:
                    raise ValueError(
                        f"Bad value at row={row_idx}, col='{name}': {row.get(name)}"
                    ) from e

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
# 4.5 Amplitude scaling in joint space (NEW)
# ============================================================

def apply_amplitude_scale(joint_name: str, v: float, traj: dict) -> float:
    """
    v: joint-space angle from CSV (absolute or delta, depending on CSV_IS_ABSOLUTE_ANGLE)
    return: scaled joint-space angle
    """
    if AMPLITUDE_SCALE == 1.0:
        return float(v)

    if SCALE_MODE == "about_zero":
        return float(AMPLITUDE_SCALE * v)

    if SCALE_MODE == "about_first_frame":
        v0 = float(traj[joint_name][0])
        return float(v0 + AMPLITUDE_SCALE * (v - v0))

    raise ValueError(f"Unknown SCALE_MODE: {SCALE_MODE}")


# ============================================================
# 4.6 CSV(joint angle) -> motor command (DIRECTION 적용)
# ============================================================

def csv_to_motor_cmd(joint_name: str, csv_value: float, init_pos_map: dict):
    did = JOINT_ID[joint_name]
    sign = dir_of(did)

    if CSV_IS_ABSOLUTE_ANGLE:
        joint_target = csv_value
    else:
        joint_target = init_pos_map[joint_name] + csv_value

    return sign * float(joint_target)


# ============================================================
# 4.7 Walking helpers: interpolation + mirroring + crossfade
# ============================================================

LEFT_RIGHT_SWAP = {
    "leg_left_ankle_roll_joint":   "leg_right_ankle_roll_joint",
    "leg_left_ankle_pitch_joint":  "leg_right_ankle_pitch_joint",
    "leg_left_knee_pitch_joint":   "leg_right_knee_pitch_joint",
    "leg_left_hip_pitch_joint":    "leg_right_hip_pitch_joint",
    "leg_left_hip_yaw_joint":      "leg_right_hip_yaw_joint",
    "leg_left_hip_roll_joint":     "leg_right_hip_roll_joint",

    "leg_right_ankle_roll_joint":  "leg_left_ankle_roll_joint",
    "leg_right_ankle_pitch_joint": "leg_left_ankle_pitch_joint",
    "leg_right_knee_pitch_joint":  "leg_left_knee_pitch_joint",
    "leg_right_hip_pitch_joint":   "leg_left_hip_pitch_joint",
    "leg_right_hip_yaw_joint":     "leg_left_hip_yaw_joint",
    "leg_right_hip_roll_joint":    "leg_left_hip_roll_joint",
}

def sample_traj_linear(traj: dict, joint_name: str, phase_frame: float):
    arr = traj[joint_name]
    T = arr.shape[0]

    x = phase_frame % T
    i0 = int(np.floor(x))
    i1 = (i0 + 1) % T
    a = float(x - i0)

    return (1.0 - a) * float(arr[i0]) + a * float(arr[i1])

def build_frame_targets(traj: dict, phase_frame: float, mirrored: bool):
    out = {}
    for j in JOINT_ORDER:
        src = LEFT_RIGHT_SWAP[j] if mirrored else j
        out[j] = sample_traj_linear(traj, src, phase_frame)
    return out

def crossfade_loop_targets(traj: dict, phase_frame: float, T: int, crossfade_frames: float, mirrored: bool):
    if crossfade_frames <= 0.0:
        return build_frame_targets(traj, phase_frame, mirrored)

    x = phase_frame % T
    if x < (T - crossfade_frames):
        return build_frame_targets(traj, phase_frame, mirrored)

    alpha = (x - (T - crossfade_frames)) / crossfade_frames  # 0..1
    alpha = float(np.clip(alpha, 0.0, 1.0))
    a = alpha * alpha * (3.0 - 2.0 * alpha)  # smoothstep

    end_targets = build_frame_targets(traj, phase_frame, mirrored)
    start_targets = build_frame_targets(traj, 0.0, mirrored)

    blended = {}
    for j in JOINT_ORDER:
        blended[j] = (1.0 - a) * end_targets[j] + a * start_targets[j]
    return blended


# ============================================================
# 5. Main
# ============================================================

def main():
    traj, T = load_csv(CSV_PATH)
    print(f"[INFO] CSV loaded: {T} frames from '{CSV_PATH}'")
    print(f"[INFO] AMPLITUDE_SCALE={AMPLITUDE_SCALE:.3f}, SCALE_MODE={SCALE_MODE}, CSV_IS_ABSOLUTE_ANGLE={CSV_IS_ABSOLUTE_ANGLE}")

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
    # Phase A: go to first CSV frame
    # --------------------------------------------------------
    print("\n[INFO] Phase A: Go to first CSV frame (with DIRECTION + amplitude scale)")
    target0_motor = {}
    for name in JOINT_ORDER:
        v0 = float(traj[name][0])
        v0 = apply_amplitude_scale(name, v0, traj)  # scaling (no-op if scale=1)
        target0_motor[name] = csv_to_motor_cmd(name, v0, init_pos)

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

    print("[INFO] Phase A done.")
    time.sleep(PRE_WALK_HOLD_SEC)

    # --------------------------------------------------------
    # Phase C: Walking
    # --------------------------------------------------------
    if not ENABLE_WALK_PHASE:
        print("[INFO] ENABLE_WALK_PHASE=False. Exiting after Phase A.")
        return

    print("\n[INFO] Phase C: WALKING start")
    print(f"[INFO] WALK_MODE={WALK_MODE}, GAIT_CYCLE_SECONDS={GAIT_CYCLE_SECONDS:.3f}s, T={T}")

    frames_per_sec = T / float(GAIT_CYCLE_SECONDS)
    crossfade_frames = (LOOP_CROSSFADE_SEC * frames_per_sec)

    start_time = time.time()
    last_log_t = 0.0

    try:
        while True:
            t = time.time() - start_time
            phase_frame = t * frames_per_sec

            if WALK_MODE == "alternate_mirror":
                cycle_idx = int(np.floor(phase_frame / T))
                mirrored = (cycle_idx % 2 == 1)
            elif WALK_MODE == "loop":
                mirrored = False
            else:
                raise ValueError(f"Unknown WALK_MODE: {WALK_MODE}")

            joint_targets = crossfade_loop_targets(
                traj=traj,
                phase_frame=phase_frame,
                T=T,
                crossfade_frames=crossfade_frames,
                mirrored=mirrored,
            )

            for name in JOINT_ORDER:
                did = JOINT_ID[name]
                bus = get_bus(did)

                csv_v = float(joint_targets[name])
                csv_v = apply_amplitude_scale(name, csv_v, traj)  # (NEW) amplitude scale
                cmd = csv_to_motor_cmd(name, csv_v, init_pos)

                bus.write_read_pdo_2(did, cmd, 0.0)

            if t - last_log_t >= 1.0:
                last_log_t = t
                cyc = (phase_frame % T) / T
                print(f"[INFO] walk t={t:6.2f}s  phase={cyc:5.2f}  mirrored={mirrored}")

            rate.sleep()

    except KeyboardInterrupt:
        print("\n[INFO] KeyboardInterrupt: stopping...")

    finally:
        print("[INFO] Set all motors to IDLE")
        for did in ALL_IDS:
            get_bus(did).set_mode(did, recoil.Mode.IDLE)
        bus_left.stop()
        bus_right.stop()
        print("[INFO] Buses stopped. Program terminated.")


if __name__ == "__main__":
    main()
