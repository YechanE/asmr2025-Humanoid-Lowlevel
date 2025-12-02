# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

"""
Keyboard Controller Module for Berkeley Humanoid Lite (Gamepad Replacement)

This module implements a keyboard-based controller that mimics the interface of
the original Se2Gamepad exactly. It ensures compatibility with calibrate_joints.py.
"""

import threading
import time
from typing import Dict
from pynput import keyboard

# 가짜 클래스: 기존 코드와의 호환성을 위해 이름만 남겨둡니다. (실제 기능은 없음)
class XInputEntry:
    pass

class Se2Gamepad:
    def __init__(self,
                 stick_sensitivity: float = 1.0,
                 dead_zone: float = 0.01,
                 ) -> None:
        self.stick_sensitivity = stick_sensitivity
        self.dead_zone = dead_zone

        self._stopped = threading.Event()
        self._listener = None
        
        # 현재 눌려 있는 키들을 저장하는 집합 (Set)
        self._keys_pressed = set()

        # [중요] 외부에서 접근하는 인터페이스 (Dictionary)
        # calibrate_joints.py가 .get()을 호출하는 대상이 바로 이것입니다.
        self.commands = {
            "velocity_x": 0.0,
            "velocity_y": 0.0,
            "velocity_yaw": 0.0,
            "mode_switch": 0,
        }

    def reset(self) -> None:
        self._keys_pressed.clear()
        self.commands = {
            "velocity_x": 0.0,
            "velocity_y": 0.0,
            "velocity_yaw": 0.0,
            "mode_switch": 0,
        }

    def stop(self) -> None:
        print("Keyboard controller stopping...")
        self._stopped.set()
        if self._listener is not None:
            self._listener.stop()

    def _on_press(self, key):
        # 키가 눌렸을 때 Set에 추가
        try:
            if hasattr(key, 'char') and key.char:
                self._keys_pressed.add(key.char.lower())
            else:
                self._keys_pressed.add(key)
        except AttributeError:
            self._keys_pressed.add(key)
            
        self._update_command_buffer()

    def _on_release(self, key):
        # 키가 떼졌을 때 Set에서 제거
        try:
            if hasattr(key, 'char') and key.char:
                self._keys_pressed.discard(key.char.lower())
            else:
                self._keys_pressed.discard(key)
        except AttributeError:
            self._keys_pressed.discard(key)
            
        self._update_command_buffer()

    def run(self) -> None:
        # 비동기 리스너 시작 (Main Loop를 방해하지 않음)
        self._listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release)
        self._listener.start()

    def _update_command_buffer(self) -> None:
        """
        키보드 상태(Set)를 읽어서 commands 딕셔너리를 업데이트합니다.
        이 과정 덕분에 외부에서는 이 객체가 키보드인지 게임패드인지 알 필요가 없습니다.
        """
        vel_x = 0.0
        vel_y = 0.0
        vel_yaw = 0.0
        mode_switch = 0

        # --- Velocity Mapping ---
        # W/S : Forward/Backward (Velocity X)
        if 'w' in self._keys_pressed: vel_x += 1.0
        if 's' in self._keys_pressed: vel_x -= 1.0

        # A/D : Left/Right (Velocity Y)
        if 'a' in self._keys_pressed: vel_y += 1.0
        if 'd' in self._keys_pressed: vel_y -= 1.0

        # Q/E : Turn Left/Right (Yaw)
        if 'q' in self._keys_pressed: vel_yaw += 1.0
        if 'e' in self._keys_pressed: vel_yaw -= 1.0

        # --- Mode Switching Mapping ---
        # calibrate_joints.py를 종료시키기 위한 트리거
        # 스페이스바 또는 x 키를 누르면 mode_switch가 1이 됨
        if keyboard.Key.space in self._keys_pressed or 'x' in self._keys_pressed:
            mode_switch = 1
        
        # 'i' : Init Mode (2)
        elif 'i' in self._keys_pressed:
            mode_switch = 2
        
        # 'r' : RL Running Mode (3)
        elif 'r' in self._keys_pressed:
            mode_switch = 3

        # [중요] 딕셔너리 값 업데이트
        # 여기가 갱신되면 calibrate_joints.py의 .get() 호출 값이 즉시 바뀝니다.
        self.commands["velocity_x"] = vel_x * self.stick_sensitivity
        self.commands["velocity_y"] = vel_y * self.stick_sensitivity
        self.commands["velocity_yaw"] = vel_yaw * self.stick_sensitivity
        self.commands["mode_switch"] = mode_switch

if __name__ == "__main__":
    # 테스트 코드
    command_controller = Se2Gamepad()
    command_controller.run()

    print("Keyboard Controller Running...")
    print("  [SPACE] or [X] -> Finish Calibration (Mode 1)")
    print("  Press CTRL+C to quit.")

    try:
        while True:
            # calibrate_joints.py가 호출하는 것과 똑같은 방식
            mode = command_controller.commands.get("mode_switch")
            vx = command_controller.commands.get("velocity_x")
            print(f"\rMode: {mode}, Vx: {vx:.2f}", end="")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt")
    
    command_controller.stop()