# ... (상단 동일)
import math
import berkeley_humanoid_lite_lowlevel.recoil as recoil

args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1_000_000)
device_id = args.id

def configure_fast_frame_rate(bus: recoil.Bus, device_id: int, fast_frame_rate: int):
    print("Rate (before):", bus._read_parameter_u32(device_id, recoil.Parameter.FAST_FRAME_FREQUENCY))
    bus._write_parameter_u32(device_id, recoil.Parameter.FAST_FRAME_FREQUENCY, fast_frame_rate)
    print("Rate (updated):", bus._read_parameter_u32(device_id, recoil.Parameter.FAST_FRAME_FREQUENCY))

def configure_gear_ratio(bus: recoil.Bus, device_id: int, gear_ratio: float):
    print("Gear Ratio (before):", bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_GEAR_RATIO))
    bus._write_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_GEAR_RATIO, gear_ratio)  # 부호 제거
    print("Gear Ratio (updated):", bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_GEAR_RATIO))

def configure_position_pd(bus: recoil.Bus, device_id: int, kp: float, kd_as_vel_kp: float):
    print("Kp (before):", bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_POSITION_KP))
    bus._write_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_POSITION_KP, kp)
    print("Kp (updated):", bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_POSITION_KP))

    # 주의: 이 구현에서 'position_kd'는 내부적으로 velocity_kp에 기록됨
    print("Vel Kp (before):", bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_VELOCITY_KP))
    bus._write_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_VELOCITY_KP, kd_as_vel_kp)
    print("Vel Kp (updated):", bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_VELOCITY_KP))

def configure_torque_limit(bus: recoil.Bus, device_id: int, torque_limit: float):
    print("Torque Limit (before):", bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_TORQUE_LIMIT))
    bus._write_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_TORQUE_LIMIT, torque_limit)
    print("Torque Limit (updated):", bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_TORQUE_LIMIT))

def configure_phase_order(bus: recoil.Bus, device_id: int, phase_order: int):
    print("Phase order (before):", bus._read_parameter_i32(device_id, recoil.Parameter.MOTOR_PHASE_ORDER))
    bus._write_parameter_i32(device_id, recoil.Parameter.MOTOR_PHASE_ORDER, phase_order)
    print("Phase order (updated):", bus._read_parameter_i32(device_id, recoil.Parameter.MOTOR_PHASE_ORDER))

def configure_position_limit(bus: recoil.Bus, device_id: int, lower_limit: float, upper_limit: float):
    print("Pos Lower (before):", bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_POSITION_LIMIT_LOWER))
    bus._write_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_POSITION_LIMIT_LOWER, lower_limit)
    print("Pos Lower (updated):", bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_POSITION_LIMIT_LOWER))

    print("Pos Upper (before):", bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_POSITION_LIMIT_UPPER))
    bus._write_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_POSITION_LIMIT_UPPER, upper_limit)
    print("Pos Upper (updated):", bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_POSITION_LIMIT_UPPER))

# ✔︎ 시그니처 수정: bandwidth, R, L 을 받아 내부에서 set_current_bandwidth 호출
def configure_current_bandwidth(bus: recoil.Bus, device_id: int, bandwidth_hz: float,
                                phase_resistance: float, phase_inductance: float):
    print("Current Kp (before):", bus._read_parameter_f32(device_id, recoil.Parameter.CURRENT_CONTROLLER_I_KP))
    print("Current Ki (before):", bus._read_parameter_f32(device_id, recoil.Parameter.CURRENT_CONTROLLER_I_KI))
    bus.set_current_bandwidth(device_id, bandwidth_hz, phase_resistance, phase_inductance)
    print("Current Kp (updated):", bus._read_parameter_f32(device_id, recoil.Parameter.CURRENT_CONTROLLER_I_KP))
    print("Current Ki (updated):", bus._read_parameter_f32(device_id, recoil.Parameter.CURRENT_CONTROLLER_I_KI))

def store_to_flash(bus: recoil.Bus, device_id: int):
    bus.store_settings_to_flash(device_id)  # 함수명 수정
    print("Settings stored to flash!")

# ===== 여기서 실제 호출(주석 해제) =====
try:
    configure_fast_frame_rate(bus, device_id, 500)
    configure_gear_ratio(bus, device_id, 1.0)
    configure_position_pd(bus, device_id, kp=0.2, kd_as_vel_kp=0.05)
    configure_torque_limit(bus, device_id, torque_limit=0.3)
    # 필요 시만:
    # configure_phase_order(bus, device_id, -1)
    # configure_position_limit(bus, device_id, -0.5*math.pi, +0.5*math.pi)
    # 모터 스펙에 맞게 R, L을 넣으세요(예시: R=0.3 Ω, L=0.3 mH):
    # configure_current_bandwidth(bus, device_id, bandwidth_hz=500.0, phase_resistance=0.3, phase_inductance=0.0003)
    # store_to_flash(bus, device_id)
finally:
    bus.stop()
