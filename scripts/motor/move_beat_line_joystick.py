import time
import numpy as np
import pygame

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil

# ================== PYGAME ==================
pygame.init()
pygame.joystick.init()

screen = pygame.display.set_mode((400, 200))
pygame.display.set_caption("JOYSTICK + MOTEUR")

if pygame.joystick.get_count() == 0:
    raise RuntimeError("Aucune manette dÃ©tectÃ©e")

js = pygame.joystick.Joystick(0)
js.init()
print("ðŸŽ® Manette dÃ©tectÃ©e :", js.get_name())

# ================== MAPPING ==================
R2_AXIS = 5
R2_THRESHOLD = 0.5
BTN_ROND = 1

# ================== MOTEUR ==================
args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)
device_id = args.id

kp = 0.2
kd = 0.005
frequency = 2.0
amplitude = 10.0
time_div = 0.5

rate = RateLimiter(frequency=200.0)

bus.write_position_kp(device_id, kp)
bus.write_position_kd(device_id, kd)
bus.write_torque_limit(device_id, 0.4)
bus.set_mode(device_id, recoil.Mode.POSITION)
bus.feed(device_id)

# ================== Ã‰TATS HIT ==================
hit_active = False
hit_start_time = 0.0

def base_sine(progress):
    return abs(np.sin(np.pi * progress) * amplitude * 2)

# ================== BEAT ==================
beat_enabled = False
beat_line = "/x----/"
beat_index = 0
beat_timestamp = time.time()

# ================== JOYSTICK ==================
r2_prev = False
rond_prev = False

def read_joystick():
    global hit_active, hit_start_time
    global beat_enabled, beat_index, beat_timestamp
    global r2_prev, rond_prev

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return False

    # ---- R2 â†’ HIT ----
    r2_val = js.get_axis(R2_AXIS)
    r2_pressed = r2_val > R2_THRESHOLD

    if r2_pressed and not r2_prev:
        hit_active = True
        hit_start_time = time.time()
        print("ðŸŽ® R2 â†’ HIT")

    r2_prev = r2_pressed

    # ---- ROND â†’ TOGGLE BEAT ----
    rond_pressed = js.get_button(BTN_ROND)

    if rond_pressed and not rond_prev:
        beat_enabled = not beat_enabled
        beat_index = 0
        beat_timestamp = time.time()
        print("ðŸŽ® ROND â†’ BEAT =", beat_enabled)

    rond_prev = rond_pressed

    return True

# ================== BEAT STEP ==================
def process_beat():
    global beat_index, beat_timestamp, hit_active, hit_start_time

    if time.time() - beat_timestamp < time_div:
        return

    beat_timestamp = time.time()

    if beat_line[beat_index] == "x":
        hit_active = True
        hit_start_time = time.time()
        print("ðŸ¥ BEAT HIT")

    beat_index = (beat_index + 1) % len(beat_line)

# ================== BOUCLE TEMPS RÃ‰EL ==================
print("\n=== MODE JOYSTICK + MOTEUR ===")
print("R2   â†’ HIT")
print("ROND â†’ TOGGLE BEAT")
print("Fermer fenÃªtre / Ctrl+C â†’ STOP\n")

running = True
try:
    while running:
        running = read_joystick()

        if beat_enabled:
            process_beat()

        # ---- calcul moteur ----
        if hit_active:
            progress = (time.time() - hit_start_time) * frequency
            if progress >= 1.0:
                hit_active = False
                target_angle = 0.0
            else:
                target_angle = base_sine(progress)
        else:
            target_angle = 0.0

        bus.write_read_pdo_2(device_id, target_angle, 0.0)

        rate.sleep()   # âš ï¸ UN SEUL rate limiter

except KeyboardInterrupt:
    pass

# ================== STOP PROPRE ==================
print("ArrÃªt moteur...")
bus.set_mode(device_id, recoil.Mode.IDLE)
bus.stop()
pygame.quit()
print("STOP")
