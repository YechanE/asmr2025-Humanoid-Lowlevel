# imu_visualize_ascii.py
# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time
import numpy as np
from loop_rate_limiters import RateLimiter

from imu_1 import RobustSerialImu  # <-- 아래 설명 참고


def quat_to_euler(qw, qx, qy, qz):
    """
    Quaternion -> roll, pitch, yaw (deg)
    ZYX 순 (yaw-pitch-roll)
    """
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.degrees(np.arctan2(sinr_cosp, cosr_cosp))

    # pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.degrees(np.arcsin(sinp))

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.degrees(np.arctan2(siny_cosp, cosy_cosp))

    return roll, pitch, yaw


def bar(value_deg, max_deg=90.0, width=30):
    """
    value_deg (-max_deg ~ +max_deg)을 0~width 사이의 막대 위치로 매핑해서
    텍스트 바 형태로 표현.
    예: |#############...............|
    """
    v = np.clip(value_deg, -max_deg, max_deg)
    # [-max, max] -> [0, width]
    pos = int((v + max_deg) / (2 * max_deg) * width)
    bar_chars = ["." for _ in range(width)]
    if 0 <= pos < width:
        bar_chars[pos] = "#"
    return "|" + "".join(bar_chars) + "|"


def heading_compass(yaw_deg):
    """
    yaw(heading)을 간단한 나침반 문자로 표시.
    """
    # 0도 기준: 북(N)
    directions = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    # -180~180 -> 0~360
    yaw_360 = (yaw_deg + 360.0) % 360.0
    idx = int((yaw_360 / 360.0) * len(directions)) % len(directions)
    return directions[idx]


def clear_screen():
    # 터미널 화면 지우고 커서 홈으로 이동
    print("\x1b[2J\x1b[H", end="")


def main():
    # 이미 IMU가 9600 baud로 동작한다고 가정 (위에서 설정 완료)
    imu = RobustSerialImu(port="/dev/ttyUSB0", baudrate=9600)
    imu.start()

    rate = RateLimiter(50)  # 50 Hz로 화면 갱신

    try:
        while True:
            acc = imu.acceleration
            gyro = imu.angular_velocity
            quat = imu.quaternion

            # 데이터 유효성 체크
            if np.allclose(acc, 0.0, atol=1e-3) and np.allclose(quat, 0.0, atol=1e-3):
                status = "NO DATA / IDLE"
            else:
                status = "ACTIVE"

            qw, qx, qy, qz = quat
            roll, pitch, yaw = quat_to_euler(qw, qx, qy, qz)
            compass = heading_compass(yaw)

            clear_screen()
            print("=== IMU ORIENTATION VISUALIZER (ASCII) ===")
            print(f"Status : {status}")
            print("")
            print(f"Acc    : {acc[0]:6.3f}  {acc[1]:6.3f}  {acc[2]:6.3f}   [g-ish]")
            print(f"Gyro   : {gyro[0]:6.2f}  {gyro[1]:6.2f}  {gyro[2]:6.2f} [deg/s]")
            print("")
            print(f"Quat   : {qw:+.3f}  {qx:+.3f}  {qy:+.3f}  {qz:+.3f}")
            print("")
            print(f"Roll   : {roll:+6.1f} deg  " + bar(roll))
            print(f"Pitch  : {pitch:+6.1f} deg  " + bar(pitch))
            print(f"Yaw    : {yaw:+6.1f} deg  " + bar(yaw))
            print(f"Heading: {compass}")
            print("")
            print("Ctrl+C to quit.")

            rate.sleep()

    except KeyboardInterrupt:
        pass
    finally:
        imu.stop()
        print("IMU stopped.")


if __name__ == "__main__":
    main()
