SERVER_NODE_NAME = "server"
CLIENT_NODE_NAME = "ui"
PID_SETUP = {
    "kpX" : 6,
    "kiX" : 0.1,
    "kdX" : 0,
    "kpW" : 5,
    "kiW" : 0.1,
    "kdW" : 10,
}
BALL_SET_POINT = {
    "distance" : 15,
    "angle" : 85,
    "x_onCamera" : 315,
    "y_onCamera" : 178,
}

OFFSET = {
    "offset_x" : -200,
    "offset_z" : 0,
    "offset_heading" : 270,
}
CAMERA_MIDDLE_POINT = {
    "x":311,
    "y":238,
}
IDLE_BALL_RADIUS = 18
IDLE_BALL_POSITION = {
    "x":320,
    "y":180
}
SPEED_CAMERA_PID = {
    "kp":1.0,
    "ki":0.1,
    "kd":0.05,
    # "setPoint":
}
ROTATION_CAMERA_PID = {
    "kp":1.0,
    "ki":0.1,
    "kd":0.05,
}
ROBOT_STATUS_TOPIC_NAME = "/nuc/RobotStatus"
NAVIGATION_SETTING_TOPIC_NAME = "/nuc/NavigationSetting"
WHEEL_TOPIC_NAME = "/nuc/Wheel"
DRIBBLE_MODULE_TOPIC_NAME = "/nuc/DribbleModule"
T2650_ODOMETRY_TOPIC_NAME = "/t265/Odometry"
