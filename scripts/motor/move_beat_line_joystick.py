# Copyright (c) 2025
import time
import numpy as np

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil

# --------- MANETTE PS4 ----------
import pygame
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise RuntimeError("Aucune manette détectée !")

js = pygame.joystick.Joystick(0)
js.init()
print("Manette détectée :", js.get_name())

R2_AXIS = 5
BTN_ROND = 1        # PS4: bouton rond
R2_THRESHOLD = 0.5
r2_previous_state = False
rond_previous_state = False
# ---------------------------------


args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)
device_id = args.id

kp = 0.2
kd = 0.005
frequency = 0.50
amplitude = 1.0
time_div = 0.5

beat_line = "/-----/-----/-----"

rate = RateLimiter(frequency=200.0)

bus.write_position_kp(device_id, kp)
bus.write_position_kd(device_id, kd)
bus.write_torque_limit(device_id, 0.2)

bus.set_mode(device_id, recoil.Mode.POSITION)
bus.feed(device_id)


# =========== MOUVEMENTS ============
def linear_move(beginning_hit):
    if (time.time() - beginning_hit <= 1/frequency * 0.5):
        return amplitude*((time.time()-beginning_hit)/(1/frequency * 0.5))
    else:
        return 2*amplitude - amplitude*((time.time()-beginning_hit)/(1/frequency * 0.5))

def base_sine_move(beginning_hit):
    return abs(np.sin(2*np.pi*(frequency/2)*(time.time()-beginning_hit)) * amplitude * 2)

def fancy_sine_move(beginning_hit):
    if time.time()-beginning_hit <= 1/frequency * 0.5:
        return abs((np.cos(2*np.pi*(frequency/2)*(time.time()-beginning_hit)) - 1) * amplitude)
    else:
        return (amplitude - (np.cos(2*np.pi*frequency*(time.time()-beginning_hit)) + 1)*amplitude*0.5)

def hit_once():
    beginning_hit = time.time()
    print("\n>>> HIT START")
    while (time.time()-beginning_hit <= (1/frequency)):
        angle = base_sine_move(beginning_hit)
        bus.write_read_pdo_2(device_id, angle, 0.0)
        rate.sleep()
    print(">>> HIT END\n")

def don_t_move(wait_time):
    begin = time.time()
    while time.time() - begin <= wait_time:
        bus.write_read_pdo_2(device_id, 0, 0.0)
        rate.sleep()
# ====================================


# ======== MODE BEAT (non bloquant) ========
beat_index = 0
beat_enabled = False
beat_timestamp = time.time()

def process_beat_step():
    """Exécute une seule étape du beat sans bloquer la manette."""
    global beat_index, beat_timestamp

    if time.time() - beat_timestamp < time_div:
        return  # attendre la prochaine cellule du beat

    beat_timestamp = time.time()  # reset timer

    symbol = beat_line[beat_index]

    if symbol == "x":   # frappe
        print("BEAT → hit_once()")
        hit_once()
    else:
        print("BEAT → silence")

    beat_index = (beat_index + 1) % len(beat_line)
# ============================================


# ========== LECTURE MANETTE ==========
def read_controller():
    global r2_previous_state, rond_previous_state, beat_enabled

    pygame.event.pump()

    # ---- R2 → hit_once ----
    r2_value = js.get_axis(R2_AXIS)
    r2_pressed = r2_value > R2_THRESHOLD

    if r2_pressed and not r2_previous_state:
        print("🎮 R2 → hit_once()")
        hit_once()

    r2_previous_state = r2_pressed

    # ---- ROND → toggle beat mode ----
    rond_pressed = js.get_button(BTN_ROND)

    if rond_pressed and not rond_previous_state:
        beat_enabled = not beat_enabled
        print("\n🎮 ROND → beat =", "ACTIVÉ" if beat_enabled else "DÉSACTIVÉ")

    rond_previous_state = rond_pressed
# ====================================



# ======== BOUCLE PRINCIPALE ==========
try:
    print("\n=== MODE MANETTE + BEAT ===")
    print("R2 = hit_once() manuel")
    print("ROND = activer/désactiver beat\n")

    while True:
        read_controller()

        if beat_enabled:
            process_beat_step()

        time.sleep(0.01)

except KeyboardInterrupt:
    pass

bus.set_mode(device_id, recoil.Mode.IDLE)
bus.stop()

