import threading
from typing import Dict
import pygame


class XInputEntry:
    # AXES PS4
    AXIS_X_L = 0        # left stick left/right
    AXIS_Y_L = 1        # left stick up/down
    AXIS_X_R = 3        # right stick left/right
    AXIS_Y_R = 4        # right stick up/down

    # Buttons PS4
    BTN_A = 0       # Croix
    BTN_B = 1       # Rond
    BTN_X = 2       # Triangle
    BTN_Y = 3       # CarrÃ©
    BTN_BUMPER_L = 4  # L1
    BTN_BUMPER_R = 5  # R1
    BTN_THUMB_L = 11  # L3
    BTN_THUMB_R = 12  # R3
    BTN_BACK = 8      # Share
    BTN_START = 9     # Options

    # D-pad
    BTN_HAT_X = "HAT_X"
    BTN_HAT_Y = "HAT_Y"


class Se2Gamepad:
    def __init__(self, stick_sensitivity=1.0, dead_zone=0.10):
        self.stick_sensitivity = stick_sensitivity
        self.dead_zone = dead_zone

        self._stopped = threading.Event()
        self._run_forever_thread = None

        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No controller detected!")

        self.js = pygame.joystick.Joystick(0)
        self.js.init()

        print("Connected:", self.js.get_name())

        self.axis_states: Dict[int, float] = {}
        self.button_states: Dict[int, int] = {}
        self.hat_states: Dict[str, int] = {"x": 0, "y": 0}

        self.commands = {
            "velocity_x": 0.0,
            "velocity_y": 0.0,
            "velocity_yaw": 0.0,
            "mode_switch": 0,
        }

    def apply_deadzone(self, v: float):
        return 0.0 if abs(v) < self.dead_zone else v

    def stop(self):
        print("Gamepad stopping...")
        self._stopped.set()

    def run(self):
        self._run_forever_thread = threading.Thread(target=self.run_forever)
        self._run_forever_thread.start()

    def run_forever(self):
        while not self._stopped.is_set():
            self.advance()

    def advance(self):
        pygame.event.pump()

        # Axes
        for axis in range(self.js.get_numaxes()):
            self.axis_states[axis] = self.js.get_axis(axis)

        # Buttons
        for btn in range(self.js.get_numbuttons()):
            self.button_states[btn] = self.js.get_button(btn)

        # D-pad
        hat_x, hat_y = self.js.get_hat(0)
        self.hat_states["x"] = hat_x
        self.hat_states["y"] = hat_y

        self._update_command_buffer()

    def _update_command_buffer(self):

        # ---- VERSION JEUX VIDÃ‰O ----
        vx   = self.apply_deadzone(self.axis_states.get(XInputEntry.AXIS_Y_L, 0))  # avant/arriÃ¨re stick gauche
        vy   = self.apply_deadzone(self.axis_states.get(XInputEntry.AXIS_X_L, 0))  # gauche/droite stick gauche
        vyaw = self.apply_deadzone(self.axis_states.get(XInputEntry.AXIS_X_R, 0))  # rotation stick droit

        self.commands["velocity_x"] = -vx
        self.commands["velocity_y"] = -vy
        self.commands["velocity_yaw"] = -vyaw

        # Mode switching
        mode = 0

        if self.button_states.get(XInputEntry.BTN_A) and self.button_states.get(XInputEntry.BTN_BUMPER_R):
            mode = 3

        if self.button_states.get(XInputEntry.BTN_A) and self.button_states.get(XInputEntry.BTN_BUMPER_L):
            mode = 2

        if self.button_states.get(XInputEntry.BTN_X) or \
           self.button_states.get(XInputEntry.BTN_THUMB_L) or \
           self.button_states.get(XInputEntry.BTN_THUMB_R):
            mode = 1

        self.commands["mode_switch"] = mode


if __name__ == "__main__":
    g = Se2Gamepad()
    g.run()

    try:
        while True:
            print(
                f"{g.commands['velocity_x']:.2f}, "
                f"{g.commands['velocity_y']:.2f}, "
                f"{g.commands['velocity_yaw']:.2f}, "
                f"mode={g.commands['mode_switch']}"
            )
    except KeyboardInterrupt:
        print("Stopped.")
        g.stop()
