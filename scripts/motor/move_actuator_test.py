# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time
import numpy as np

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil


args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)

device_id = args.id

#0.2
kp = 0.2  
kd = 0.01

frequency = 1.0  # motion frequency is 1 Hz
amplitude = 2.0  # motion amplitude is 1 rad

rate = RateLimiter(frequency=200.0) 

print(f"Reading default values for ID {device_id}...")
pos_kp = bus.read_position_kp(device_id)
pos_kd = bus.read_position_kd(device_id)
torque_limit = bus.read_torque_limit(device_id)
print("==="*5)
print(f"Default Position KP: {pos_kp}")
print(f"Default Position KD: {pos_kd}")
print(f"Default Torque Limit: {torque_limit}")
print("==="*5)

bus.write_position_kp(device_id, kp)
bus.write_position_kd(device_id, kd)
bus.write_torque_limit(device_id, 1.5)

bus.set_mode(device_id, recoil.Mode.POSITION)
bus.feed(device_id)

time.sleep(0.001)
pos_kp = bus.read_position_kp(device_id)
pos_kd = bus.read_position_kd(device_id)
torque_limit = bus.read_torque_limit(device_id)
print("==="*5)
print(f"Default Position KP: {pos_kp}")
print(f"Default Position KD: {pos_kd}")
print(f"Default Torque Limit: {torque_limit}")
print("==="*5)
time.sleep(0.01)
##
print("Reading initial position...")
initial_position, _ = bus.write_read_pdo_2(device_id, 0.0, 0.0)
if initial_position is None:
    print("Failed to read initial position.")
    initial_position = 0.0

print(f"Initial Position = {initial_position:.3f} rad")

##

try:
    while True: 
        target_angle = initial_position + np.sin(2 * np.pi * frequency * time.time()) * amplitude

        measured_position, measured_velocity = bus.write_read_pdo_2(device_id, target_angle, 0.0)
        if measured_position is not None and measured_velocity is not None:
            print(f"Measured pos: {measured_position:.3f} \tvel: {measured_velocity:.3f}")
            print(f"target pos: {target_angle:.3f}")


        rate.sleep()

except KeyboardInterrupt:
    pass

bus.set_mode(device_id, recoil.Mode.IDLE)
bus.stop()
