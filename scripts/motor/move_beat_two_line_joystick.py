import time
import numpy as np
import pygame

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil

# ============================================================
# 0) USER CONFIG
# ============================================================
# 조이스틱 매핑 (환경에 따라 다를 수 있음)
L2_AXIS = 2          # 보통 PS 계열에서 L2가 axis 2인 경우가 많음
R2_AXIS = 5          # 보통 PS 계열에서 R2가 axis 5인 경우가 많음
TRIGGER_THRESHOLD = 0.5

# 모터 제어 파라미터
KP = 0.2
KD = 0.005
TORQUE_LIMIT = 0.2

HIT_FREQUENCY = 2.0   # HIT 1회 길이 = 1/HIT_FREQUENCY (초)
AMPLITUDE = 10.0      # base_sine 내부에서 *2가 들어가면 피크가 ~20 근처까지 감 (주의)

CONTROL_HZ = 200.0

# CAN 채널 / ID 매핑
# can2의 id=1은 R2로, can3의 id=1은 L2로 HIT
CAN2_CHANNEL = "can2"
CAN3_CHANNEL = "can3"
DEVICE_ID_CAN0 = 1
DEVICE_ID_CAN1 = 1

# ============================================================
# 1) PYGAME INIT
# ============================================================
pygame.init()
pygame.joystick.init()

screen = pygame.display.set_mode((400, 200))
pygame.display.set_caption("DUAL CAN (can0/can1) + JOYSTICK (R2/L2)")

if pygame.joystick.get_count() == 0:
    raise RuntimeError("Aucune manette détectée")

js = pygame.joystick.Joystick(0)
js.init()
print("🎮 Manette détectée :", js.get_name())
print(f"num_axes={js.get_numaxes()}, num_buttons={js.get_numbuttons()}")

# ============================================================
# 2) BUS INIT (can0, can1)
# ============================================================
bus0 = recoil.Bus(channel=CAN2_CHANNEL, bitrate=1000000)
bus1 = recoil.Bus(channel=CAN3_CHANNEL, bitrate=1000000)

def setup_motor(bus, device_id):
    bus.write_position_kp(device_id, KP)
    bus.write_position_kd(device_id, KD)
    bus.write_torque_limit(device_id, TORQUE_LIMIT)
    bus.set_mode(device_id, recoil.Mode.POSITION)
    bus.feed(device_id)

setup_motor(bus0, DEVICE_ID_CAN0)
setup_motor(bus1, DEVICE_ID_CAN1)

# ============================================================
# 3) HIT STATE (각 모터별로 따로)
# ============================================================
hit0_active = False
hit1_active = False
hit0_start_time = 0.0
hit1_start_time = 0.0

l2_prev = False
r2_prev = False

rate = RateLimiter(frequency=CONTROL_HZ)

def base_sine(progress):
    # progress: 0~1
    # 0 -> peak -> 0 형태의 펄스
    return abs(np.sin(np.pi * progress) * AMPLITUDE * 2)

def read_trigger(axis_index):
    # pygame에서 axis 값 읽기
    # 컨트롤러에 따라 범위/부호가 다를 수 있음
    return js.get_axis(axis_index)

def compute_target(hit_active, hit_start_time):
    if not hit_active:
        return 0.0, False, hit_start_time

    progress = (time.time() - hit_start_time) * HIT_FREQUENCY
    if progress >= 1.0:
        return 0.0, False, hit_start_time
    return base_sine(progress), True, hit_start_time

print("\n=== DUAL CAN CONTROL ===")
print(f"R2(axis {R2_AXIS}) -> can0:{DEVICE_ID_CAN0}")
print(f"L2(axis {L2_AXIS}) -> can1:{DEVICE_ID_CAN1}")
print("Close window / Ctrl+C -> STOP\n")

running = True
try:
    while running:
        # ---- pygame event pump ----
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # ---- Read R2/L2 ----
        r2_val = read_trigger(R2_AXIS)
        l2_val = read_trigger(L2_AXIS)

        r2_pressed = (r2_val > TRIGGER_THRESHOLD)
        l2_pressed = (l2_val > TRIGGER_THRESHOLD)

        # ---- Rising edge: R2 -> HIT for bus0 ----
        if r2_pressed and not r2_prev:
            hit0_active = True
            hit0_start_time = time.time()
            print("🎮 R2 -> HIT on can0")

        # ---- Rising edge: L2 -> HIT for bus1 ----
        if l2_pressed and not l2_prev:
            hit1_active = True
            hit1_start_time = time.time()
            print("🎮 L2 -> HIT on can1")

        r2_prev = r2_pressed
        l2_prev = l2_pressed

        # ---- Compute targets ----
        target0, hit0_active, hit0_start_time = compute_target(hit0_active, hit0_start_time)
        target1, hit1_active, hit1_start_time = compute_target(hit1_active, hit1_start_time)

        # ---- Send PDOs ----
        bus0.write_read_pdo_2(DEVICE_ID_CAN0, target0, 0.0)
        bus1.write_read_pdo_2(DEVICE_ID_CAN1, target1, 0.0)

        rate.sleep()

except KeyboardInterrupt:
    pass
finally:
    print("Arrêt moteur...")
    try:
        bus0.set_mode(DEVICE_ID_CAN0, recoil.Mode.IDLE)
    except Exception:
        pass
    try:
        bus1.set_mode(DEVICE_ID_CAN1, recoil.Mode.IDLE)
    except Exception:
        pass

    try:
        bus0.stop()
    except Exception:
        pass
    try:
        bus1.stop()
    except Exception:
        pass

    pygame.quit()
    print("STOP")
