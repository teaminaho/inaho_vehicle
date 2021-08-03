import math
import os


class JoystickControllerConfig:
    def __init__(self):
        self.publish_rate_hz = int(os.getenv("JS_CONT_PUBLISH_RATE_HZ", 20))
        self.receive_buffer_size = int(
            os.getenv("JS_CONT_RECEIVE_BUFFER_SIZE", 4))
        self.velocity_buffer_size = int(
            os.getenv("JS_CONT_VELOCITY_BUFFER_SIZE", 4))
        self.velocity_limit = float(os.getenv("JS_CONT_VELOCITY_LIMIT", 0.25))
        self.dash_velocity_limit = float(
            os.getenv("JS_CONT_DASH_VELOCITY_LIMIT", 0.4))
        self.angular_limit = float(
            os.getenv("JS_CONT_ANGULAR_LIMIT", math.pi / 6.0))
        self.lin_accel_limit = float(os.getenv("JS_CONT_LIN_ACCEL_LIMIT",
                                               0.25))
        self.ang_accel_limit = float(
            os.getenv("JS_CONT_ANG_ACCEL_LIMIT", math.pi / 6.0))
        self.lin_vel_index = int(os.getenv("JS_CONT_STICK_LIN_VEL_INDEX", 3))
        self.ang_vel_index = int(os.getenv("JS_CONT_STICK_ANG_VEL_INDEX", 2))
        self.stick_x_invert = os.getenv("JS_CONT_STICK_X_INVERT", "0") == "1"
        self.stick_y_invert = os.getenv("JS_CONT_STICK_Y_INVERT", "0") == "1"
        self.unlock_button_index = int(
            os.getenv("JS_CONT_UNLOCK_BUTTON_INDEX", 7))
        self.dash_button_index = int(os.getenv("JS_CONT_DASH_BUTTON_INDEX", 2))
