import copy
import math
from collections import deque
from time import time

AXIS_TO_BUTTON_OFFSET = 100  # type: int
AXIS_TO_BUTTON_THRESHOLD = 0.5  # type: float


class JoystickController:
    def __init__(self, config):
        self._config = copy.deepcopy(config)
        self._rcv_buf = deque(maxlen=self._config.receive_buffer_size)
        self._vel_buf = deque(maxlen=self._config.velocity_buffer_size)
        self._velocity_axis_index = self._config.lin_vel_index
        self._angular_axis_index = self._config.ang_vel_index
        self._vel_buf_time = ((self._config.velocity_buffer_size * 1.0) /
                              (self._config.publish_rate_hz * 1.0))
        self._watchdog_timer_interval = 0.5
        self._last_updated_timestamp = time()

    def append_data(self, data):
        self._rcv_buf.append(data)
        self._last_updated_timestamp = time()

    def _average_velocity(self, data):
        if data is None:
            return 0.0
        return math.fsum([x[0] if x is not None else 0.0
                          for x in data]) / self._config.velocity_buffer_size

    def _average_angular(self, data):
        if data is None:
            return 0.0
        return math.fsum([x[1] if x is not None else 0.0
                          for x in data]) / self._config.velocity_buffer_size

    def _calc_velocity_command(self, data_rcvd, data, vel_limit):
        axis_idx = self._velocity_axis_index
        accl_limit = self._config.lin_accel_limit
        t = self._vel_buf_time

        if data_rcvd is not None:
            rcvd_vel_cmd = data_rcvd.axes[axis_idx]
        else:
            rcvd_vel_cmd = 0.0
        # convert from no unit value to value in m/s
        rcvd_vel_cmd *= vel_limit

        avg_vel = self._average_velocity(data)
        accl = (rcvd_vel_cmd - avg_vel) / t

        if accl_limit < abs(accl):
            # limit velocity if acceleration limit exceeded
            sign = 1.0 if accl >= 0.0 else -1.0
            vel_cmd = avg_vel + (accl_limit * t * sign)
        else:
            vel_cmd = rcvd_vel_cmd
        return vel_cmd

    def _calc_angular_command(self, data_rcvd, data):
        ang_axis_idx = self._angular_axis_index
        ang_limit = self._config.angular_limit
        accl_limit = self._config.ang_accel_limit
        t = self._vel_buf_time

        if data_rcvd is not None:
            rcvd_ang_cmd = data_rcvd.axes[ang_axis_idx]
        else:
            rcvd_ang_cmd = 0.0
        # convert from no unit value to value in rad/s
        rcvd_ang_cmd *= ang_limit

        avg_ang = self._average_angular(data)
        accl = (rcvd_ang_cmd - avg_ang) / t

        if accl_limit < abs(accl):
            sign = 1.0 if accl >= 0.0 else -1.0
            ang_cmd = avg_ang + (accl_limit * t * sign)
        else:
            ang_cmd = rcvd_ang_cmd
        return ang_cmd

    def read_data(self):
        if (time() - self._last_updated_timestamp) >= self._watchdog_timer_interval:
            self._rcv_buf.clear()
            self._vel_buf.clear()
            ret = (0.0, 0.0)
            self._vel_buf.append(ret)
            return ret

        try:
            data_rcvd = self._rcv_buf.popleft()
        except IndexError:
            data_rcvd = None

        data = list(self._vel_buf)
        unlock_btn_idx = self._config.unlock_button_index
        dash_btn_idx = self._config.dash_button_index

        if data_rcvd is not None:
            unlock_button = data_rcvd.buttons[unlock_btn_idx] == 1

            if dash_btn_idx < AXIS_TO_BUTTON_OFFSET:
                # As button
                dash_button = data_rcvd.buttons[dash_btn_idx] == 1
            else:
                # Use Axis as a button
                dash_btn_positive_side = dash_btn_idx % 2 == 0
                dash_axis_value = data_rcvd.axes[int((dash_btn_idx - AXIS_TO_BUTTON_OFFSET) / 2)]
                if dash_btn_positive_side:
                    dash_button = dash_axis_value > 0 and abs(dash_axis_value) >= AXIS_TO_BUTTON_THRESHOLD
                else:
                    dash_button = dash_axis_value < 0 and abs(dash_axis_value) >= AXIS_TO_BUTTON_THRESHOLD

        else:
            unlock_button = False
            dash_button = False

        if dash_button:
            vel_limit = self._config.dash_velocity_limit
        else:
            vel_limit = self._config.velocity_limit

        if unlock_button:
            vel_cmd = self._calc_velocity_command(data_rcvd, data, vel_limit)
            angl_cmd = self._calc_angular_command(data_rcvd, data)
        else:
            vel_cmd = self._calc_velocity_command(None, data, vel_limit)
            angl_cmd = self._calc_angular_command(None, data)

        ret = (vel_cmd, angl_cmd)
        self._vel_buf.append(ret)
        return ret
