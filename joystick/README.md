# Joystick package

## js_controller

`js_controller` reads input values from js_perception and interpret them.


### Requirement

Python 3.8


### Environment Variables

The js_controller requires to be set the following environment variables.

| Name |  Default value | Description |
| ---- | -------------- | ----------- |
| JS_CONT_PUBLISH_RATE_HZ | 20 | Publish values in times/sec |
| JS_CONT_RECEIVE_BUFFER_SIZE | 4 | Receive buffer size. approx 0.2 sec in 20Hz |
| JS_CONT_VELOCITY_BUFFER_SIZE | 4 | Buffer size to limit velocity acceleration. approx 0.2 sec in 20Hz |
| JS_CONT_VELOCITY_LIMIT | 0.25 | Limits velocity command in m/sec |
| JS_CONT_DASH_VELOCITY_LIMIT | 0.4 | Limits velocity in dash mode, in m/sec |
| JS_CONT_ANGULAR_LIMIT | 0.5236 | Limits turn rate in rad/sec |
| JS_CONT_LIN_ACCEL_LIMIT | 0.25 | Limits linear velocity acceleration in m/sec^2 |
| JS_CONT_ANG_ACCEL_LIMIT | 0.5236 | Limits turn rate acceleration in rad/sec^2 |
| JS_CONT_STICK_LIN_VEL_INDEX | 3 | Linear velocity control stick index. Default value is for JoyCon(R) |
| JS_CONT_STICK_ANG_VEL_INDEX | 2 | Turn rate control stick index. Default value is for JoyCon(R) |
| JS_CONT_STICK_X_INVERT | 0 | 1 to invert X axis value. Default value is 0 but 1 is required for JoyCon(R) |
| JS_CONT_STICK_Y_INVERT | 0 | 1 to invert Y axis value.  Default value is 0 for JoyCon(R)|
| JS_CONT_UNLOCK_BUTTON_INDEX | 7 | Joystick output unlock button index. Default value is for JoyCon(R) ZR button |
| JS_CONT_DASH_BUTTON_INDEX | 2 | Dash mode enable button index. Default value is for JoyCon(R) B button |


### Run js_controller

Run js_controller:

```shell
./scripts/js_controller.py
```

Note that you must set environment variables before run the script.


### Topics

| Topic | Action | Type |
| ----- | ------ | ---- |
| `/js_signal` | Subscribe | `sensor_msgs/Joy` |
| `/vehicle/cmd_vel` | Publish | `geometry_msgs/Twist` |
