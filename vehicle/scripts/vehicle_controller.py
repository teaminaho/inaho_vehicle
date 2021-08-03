#!/usr/bin/env python3
from enum import IntEnum
from typing import Dict
import rospy

# ROS standard messages
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

# ROS custom messages
from inaho_vehicle_msgs.msg import MotorStatus


# Enum defines
class WheelSide(IntEnum):
    L = +1
    R = -1


class CameraSide(IntEnum):
    L = +1
    R = -1


# Type synonims
SignedPwm = Int16
Rpm = Int16

# Constants
HEALTH_CHECK_FREQ = 20  # [Hz]
HEALTH_TIMEOUT_SECS = 2  # [s]
WAIT_MOTOR_TIMEOUT_SECS = 20  # [s]
COUNT_PER_REV = 30
COUNT_TO_RPM = HEALTH_CHECK_FREQ * 60 / COUNT_PER_REV


# Functions
def as_signed_pwm(cmd_vel: Twist) -> Dict[WheelSide, SignedPwm]:
    v_lin = cmd_vel.linear.x    # range: (-1, +1)
    v_ang = cmd_vel.angular.z   # range: (-1, +1)
    return {
        side: SignedPwm(int(round((v_lin * side - v_ang) * 255)))
        for side in WheelSide
    }


# Classes
class VehicleController(object):
    def __init__(self) -> None:
        # Initialize state variables
        self._health_counter = {WheelSide.L: -1, WheelSide.R: -1}

        # Prepare ROS publishers
        self._pub_motor_cmd = {
            WheelSide.L: rospy.Publisher('/vehicle/motor_l/cmd', SignedPwm, queue_size=1),
            WheelSide.R: rospy.Publisher('/vehicle/motor_r/cmd', SignedPwm, queue_size=1),
        }
        self._pub_motor_rpm = {
            WheelSide.L: rospy.Publisher('/vehicle/motor_l/rpm', Rpm, queue_size=1),
            WheelSide.R: rospy.Publisher('/vehicle/motor_r/rpm', Rpm, queue_size=1),
        }

        # Register ROS subscribers
        rospy.Subscriber('/vehicle/motor_l/ret', MotorStatus, self._motor_ret_cb, WheelSide.L)
        rospy.Subscriber('/vehicle/motor_r/ret', MotorStatus, self._motor_ret_cb, WheelSide.R)
        rospy.Subscriber('/vehicle/cmd_vel', Twist, self._cmd_vel_cb)

        # Wait for hardware ready
        self._wait_motor()

    def _cmd_vel_cb(self, cmd_vel: Twist) -> None:
        for side, pwm in as_signed_pwm(cmd_vel).items():
            self._pub_motor_cmd[side].publish(pwm)

    def _motor_ret_cb(self, ret: MotorStatus, side: WheelSide) -> None:
        if not ret.alarm:
            self._health_counter[side] = 0
        self._pub_motor_rpm[side].publish(Rpm(data=round(ret.count * COUNT_TO_RPM)))

    def _check_health(self) -> bool:
        for side in WheelSide:
            if self._health_counter[side] > HEALTH_TIMEOUT_SECS * HEALTH_CHECK_FREQ:
                rospy.logerr(f'ERROR : motor response timeout (side: {side.name})')
                self._health_counter = {WheelSide.L: -1, WheelSide.R: -1}
                return False
            else:
                self._health_counter[side] += 1
        return True

    def _wait_motor(self) -> bool:
        rospy.loginfo('Waiting for motor response...')
        count_secs = 0

        while (-1 in self._health_counter.values() and not rospy.is_shutdown()):
            for side in WheelSide:
                self._pub_motor_cmd[side].publish(SignedPwm(data=0))
            rospy.sleep(1.)
            count_secs += 1

            if count_secs > WAIT_MOTOR_TIMEOUT_SECS:
                rospy.logerr('Timeout to wait motor response')
                return False
            if not (-1 in self._health_counter.values()):
                rospy.loginfo('Succeeded to reseive motor responses')
                return True

    def run(self) -> None:
        rate = rospy.Rate(HEALTH_CHECK_FREQ)
        while not rospy.is_shutdown():
            self._is_healthy = self._check_health()
            if not self._is_healthy:
                if not self._wait_motor():
                    rospy.signal_shutdown("timeout")
                    return
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('vehicle_controller')
    vehicle_controller = VehicleController()
    rospy.loginfo("vehicle_controller is initialized")
    vehicle_controller.run()
    rospy.spin()
