#!/usr/bin/env python3

import rospy
import rosgraph
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3
from js_controller_config import JoystickControllerConfig
from joystick_controller import JoystickController

config = None
js_con = None
publisher = None


def check_master(ev):
    if not rospy.is_shutdown() and not rosgraph.is_master_online():
        rospy.logerr('Unable to communicate with master')
        rospy.signal_shutdown('Communication with master lost')


def read_joystick(ev):
    global config
    global js_con
    global publisher

    if publisher is None or js_con is None or config is None:
        rospy.logfatal(
            'Fatal Internal Error: Some required object not initialized')
        rospy.signal_shutdown('Fatal Internal Error')
        return

    if rospy.is_shutdown():
        return

    try:
        data = js_con.read_data()
        if data is not None:
            lin_vel = data[0] * (-1.0 if config.stick_y_invert else 1.0)
            ang_vel = data[1] * (-1.0 if config.stick_x_invert else 1.0)

            lin = Vector3(x=lin_vel, y=0.0, z=0.0)
            ang = Vector3(x=0.0, y=0.0, z=ang_vel)
            tw = Twist(linear=lin, angular=ang)

            publisher.publish(tw)

    except Exception as e:
        # Catch all exceptions and terminate process with error.
        # Doing exit and restart is quick way to recover.
        rospy.logerr(e)
        rospy.signal_shutdown('Error in processing Joystick signals')


if __name__ == '__main__':
    config = JoystickControllerConfig()
    js_con = JoystickController(config)

    rospy.init_node('js_controller')
    rospy.Subscriber('/js_signal', Joy, js_con.append_data, queue_size=1)
    publisher = rospy.Publisher('/vehicle/cmd_vel', Twist, queue_size=1)

    rospy.Timer(rospy.Duration(1.0 / 4.0), check_master)
    rospy.Timer(rospy.Duration(1.0 / config.publish_rate_hz), read_joystick)

    rospy.spin()
