# scripts/diag_check_phase_order.py
import time
import berkeley_humanoid_lite_lowlevel.recoil as recoil

def main():
    args = recoil.util.get_args()
    bus = recoil.Bus(channel=args.channel, bitrate=1000000)
    dev = args.id

    # 진폭 작게, 저속 검증
    bus.write_velocity_kp(dev, 0.05)
    bus.write_velocity_ki(dev, 0.01)
    bus.write_current_limit(dev, 3.0)
    bus.write_torque_limit(dev, 0.8)

    bus.set_mode(dev, recoil.Mode.VELOCITY)
    time.sleep(1)

    p0 = bus.read_position_measured(dev)

    # +방향 2초
    bus.write_velocity_target(dev, 0.2)  # +0.2 rad/s
    time.sleep(2.0)
    p_plus = bus.read_position_measured(dev)

    # 정지 1초
    bus.write_velocity_target(dev, 0.0)
    time.sleep(1.0)

    # -방향 2초
    bus.write_velocity_target(dev, -0.2)
    time.sleep(2.0)
    p_minus = bus.read_position_measured(dev)

    # 정지
    bus.write_velocity_target(dev, 0.0)
    bus.set_mode(dev, recoil.Mode.IDLE)
    bus.stop()

    dp_plus  = (p_plus  - p0)
    dp_minus = (p_minus - p_plus)

    print(f"Δpos(+0.2 rad/s, 2s) ≈ {dp_plus:+.3f} rad")
    print(f"Δpos(-0.2 rad/s, 2s) ≈ {dp_minus:+.3f} rad")

    # 판정 지침:
    # +명령에서 양(+)으로 증가, -명령에서 음(-)으로 감소해야 정상.
    # 반대로 나오면 MOTOR_PHASE_ORDER가 반대일 가능성 큼.

if __name__ == "__main__":
    main()
