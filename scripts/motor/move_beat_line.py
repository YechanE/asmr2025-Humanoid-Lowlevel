# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time
import numpy as np

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil


args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)

device_id = args.id

kp = 0.2
kd = 0.005

frequency = 1.0  # motion frequency is 1 Hz        __ WRIST = 0.5
amplitude = 1.0  # motion amplitude is 1 rad       __ WRIST = 30

rate = RateLimiter(frequency=200.0)


bus.write_position_kp(device_id, kp)
bus.write_position_kd(device_id, kd)
bus.write_torque_limit(device_id, 0.2)

bus.set_mode(device_id, recoil.Mode.POSITION)
bus.feed(device_id)


beat_line = "/x----/" #-----/-----/-xx--/-----/xxx--/-----/-----/-----/-----/-----/-----/-----/-----/
time_div = 0.5

def linear_move(beginning_hit):
	if (time.time() - beginning_hit <= 1/frequency * 0.5):
		target_angle = amplitude*((time.time()-beginning_hit)/(1/frequency * 0.5))
				
	else:
		target_angle = 2*amplitude - amplitude*( ((time.time()-beginning_hit)/(1/frequency * 0.5)))
	return target_angle

def base_sine_move(beginning_hit):
	target_angle = abs(np.sin(2 * np.pi * frequency/2 * (time.time()-beginning_hit) )* amplitude*2)  #frequency divided by 2 since wew only do half of the way, amplitude times 2
	return target_angle

def fancy_sine_move(beginning_hit):
	
	if time.time()-beginning_hit <= 1/frequency * 0.5 :
	
		target_angle = abs((np.cos(2 * np.pi * frequency/2 * (time.time()-beginning_hit) ) -1 )* amplitude)
		print("FIRST PHASE")
		print(target_angle)
	else :
		target_angle = ( amplitude - (np.cos(2 * np.pi * frequency * (time.time()-beginning_hit) ) +1 )* amplitude*0.5 )
		print("SECOND PHASE")
		print(target_angle)

	return target_angle

def hit_once():
	beginning_hit = time.time()
	while ( time.time()-beginning_hit <= (1/frequency)):
		
		print("MOVING")
		
		
		#####--------------------------LINEAR_MOVEMENT------------------
		
		#target_angle = linear_move(beginning_hit)
		
			
		####-------------------------BASIC_SINE_MOVEMENT----------------------

		target_angle = base_sine_move(beginning_hit)
		
		####-------------------------FANCY_SINE_MOVEMENT----------------------
		
		#target_angle = fancy_sine_move(beginning_hit)

		measured_position, measured_velocity = bus.write_read_pdo_2(device_id, target_angle, 0.0)
		if measured_position is not None and measured_velocity is not None:
			print(f"Measured pos: {measured_position:.3f} \tvel: {measured_velocity:.3f}")	

		rate.sleep()
	return

def don_t_move(wait_time):
	begin_wait = time.time()
	while (time.time() - begin_wait <= wait_time) :
		measured_position, measured_velocity = bus.write_read_pdo_2(device_id, 0, 0.0)
		#target_angle = measured_position
		if measured_position is not None and measured_velocity is not None:
			print(f"Measured pos: {measured_position:.3f} \tvel: {measured_velocity:.3f}")
		rate.sleep()
	return

def read_beat():
	for wut in beat_line:
		if wut == "/":
			pass
		elif wut == "x":
			current_beat=time.time()
			hit_once()
			don_t_move( time_div - ( time.time() - current_beat ) )
		else :
			don_t_move(time_div)
			

try:
	while True:
		read_beat()
			
			
except KeyboardInterrupt:
	pass

bus.set_mode(device_id, recoil.Mode.IDLE)
bus.stop()
