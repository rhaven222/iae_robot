import pygame


class PS5Controller:
    LEFT_X = 0
    LEFT_Y = 1
    RIGHT_X = 3
    RIGHT_Y = 4
    L2_AXIS = 2
    R2_AXIS = 5

    DPAD_HAT = 0

    X_BUTTON = 0
    CIRCLE_BUTTON = 1
    TRIANGLE_BUTTON = 2
    SQUARE_BUTTON = 3
    L1_BUTTON = 4
    R1_BUTTON = 5

    DEADZONE_LEFT_X = 0.2
    DEADZONE_LEFT_Y = 0.2
    DEADZONE_RIGHT_X = 0.2
    DEADZONE_RIGHT_Y = 0.2

    def __init__(self, joystick_index=0):
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No controller detected")

        self.js = pygame.joystick.Joystick(joystick_index)
        self.js.init()

    def pump(self):
        pygame.event.pump()

    @staticmethod
    def apply_deadzone(value, deadzone):
        if abs(value) < deadzone:
            return 0.0
        return value

    @staticmethod
    def clamp(value, low=-1.0, high=1.0):
        return max(low, min(high, value))
    
    def get_state(self):
        self.pump()
        return {
            "lx": self.apply_deadzone(self.js.get_axis(self.LEFT_X), self.DEADZONE_LEFT_X),
            "ly": self.apply_deadzone(self.js.get_axis(self.LEFT_Y), self.DEADZONE_LEFT_Y),
            "rx": self.apply_deadzone(self.js.get_axis(self.RIGHT_X), self.DEADZONE_RIGHT_X),
            "ry": self.apply_deadzone(self.js.get_axis(self.RIGHT_Y), self.DEADZONE_RIGHT_Y),
            "l2": self.js.get_axis(self.L2_AXIS),
            "r2": self.js.get_axis(self.R2_AXIS),
            "hat": self.js.get_hat(self.DPAD_HAT),
            "l1": bool(self.js.get_button(self.L1_BUTTON)),
            "r1": bool(self.js.get_button(self.R1_BUTTON)),
        }

    def get_drive(self):
        self.pump()

        lx = self.apply_deadzone(
            self.js.get_axis(self.LEFT_X),
            self.DEADZONE_LEFT_X
        )
        ly = self.apply_deadzone(
            self.js.get_axis(self.LEFT_Y),
            self.DEADZONE_LEFT_Y
        )

        forward = -ly
        turn = lx

        DRIVE_SPEED = 0.6
        TURN_SPEED = 0.4

        left_motor = self.clamp(forward * DRIVE_SPEED - turn * TURN_SPEED)
        right_motor = self.clamp(forward * DRIVE_SPEED + turn * TURN_SPEED)

        return left_motor, right_motor

    def get_camera(self):
        self.pump()

        rx = self.apply_deadzone(self.js.get_axis(self.RIGHT_X), self.DEADZONE_RIGHT_X)
        ry = self.apply_deadzone(self.js.get_axis(self.RIGHT_Y), self.DEADZONE_RIGHT_Y)

        return -rx, -ry

    def get_dpad(self):
        self.pump()
        return self.js.get_hat(self.DPAD_HAT)

    def r2_pressed(self):
        self.pump()
        return self.js.get_axis(self.R2_AXIS) > 0

    def l1_pressed(self):
        self.pump()
        return bool(self.js.get_button(self.L1_BUTTON))

    def r1_pressed(self):
        self.pump()
        return bool(self.js.get_button(self.R1_BUTTON))

    def x_pressed(self):
        self.pump()
        return bool(self.js.get_button(self.X_BUTTON))

    def triangle_pressed(self):
        self.pump()
        return bool(self.js.get_button(self.TRIANGLE_BUTTON))

    def close(self):
        pygame.quit()