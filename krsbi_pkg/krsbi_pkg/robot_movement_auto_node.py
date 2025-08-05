#!/usr/bin/env python3
import math
import time
import heapq
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float32, String
from krsbi_pkg.constants import CAMERA_MIDDLE_POINT
from krsbi_interfaces.msg import Wheel, DribbleModule, NavigationSetting, RobotStatus, ControlOutput, Coordinate, RobotMode, SetPoint, BallPositionBasedOnCamera

class PID():
    def __init__(self, kp, ki, kd, alpha=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.alpha = alpha

        self.proportional = 0
        self.integral = 0
        self.derivative = 0
        self.prev_derivative = 0
        self.last_error = 0

        # self.start_time = rospy.get_time()
        # self.last_time = 0

    # def __call__(self, error, start_time):
    #     dt = rospy.get_time() - (self.last_time if self.last_time is not None else self.start_time)
    #     d_error = error - self.last_error

    #     print(f"timer {time.time() - start_time}")
    #     if (time.time() - start_time) < 1 :
    #         self.integral = 0

    #     self.proportional = self.kp * error
    #     self.integral += self.ki * error * dt
    #     self.derivative = self.kd * d_error / dt
    #     print(f"+ Proportional : {self.proportional}\n+ Integral : {self.integral}\n+ Derivative : {self.derivative}\n\n")

    #     self.last_error = error
    #     self.last_time = rospy.get_time()

    #     return self.proportional + self.integral + self.derivative
    
    def __call__(self, error):
        self.integral += 0 #error
        self.derivative = 0 #error - self.last_error
        self.proportional = self.kp * error
        D_Filter = self.alpha * self.prev_derivative + (1 - self.alpha) * self.derivative
        derivative_final = self.kd * D_Filter

        # print(f"+ Proportional : {self.proportional}\n+ Integral : {self.integral}\n+ Derivative : {derivative_final}\n Error : {error}\n\n")

        self.last_error = error
        self.prev_derivative = derivative_final
        output = (self.proportional) + (self.ki * self.integral) + (derivative_final)
        return output

class RobotMovementAutoNode(Node):
    def __init__(self):
        super().__init__("robot_movement_auto_node")

        # Publisher
        self.pubPWM = self.create_publisher(Wheel, 'Wheel', 10)
        self.pubKickerStatus = self.create_publisher(String, '/rosserial/Kicker', 10)
        self.pubSetPoint = self.create_publisher(SetPoint, 'SetPoint', 10)
        self.pubCatchBall = self.create_publisher(String, 'catchBall', 10)
        self.pubGripper = self.create_publisher(DribbleModule, 'DribbleModule', 10)
        self.pubBallPosition = self.create_publisher(BallPositionBasedOnCamera, '/rosserial/BallPositionBasedOnCamera', 10)

        # Subscriber
        # self.create_subscription(RealsenseT265, '/t265/Odometry', self.t265Callback, 10)
        self.create_subscription(NavigationSetting, 'NavigationSetting', self.navigationSettingCallback, 10)
        self.create_subscription(RobotStatus, 'RobotStatus', self.robotStatusCallback, 10)
        self.create_subscription(ControlOutput, 'ControlOutput', self.controlOutputCallback, 10)
        self.create_subscription(Coordinate, 'Coordinate', self.coordinateCallback, 10)
        self.create_subscription(Float32, '/rosserial/BNO', self.headingCallback, 10)
        self.create_subscription(Float32, 'HeadingSetPoint', self.headingSetPointCallback, 10)
        self.create_subscription(RobotMode, 'robotMode', self.robotModeCallback, 10)
        self.create_subscription(BallPositionBasedOnCamera, '/rosserial/BallPosition', self.ballPositionBasedOnCameraCallback, 10)
        self.create_subscription(PoseArray, '/yolov5/ball', self.ballCoordinateCallback, 10)

        self.wheel = Wheel()
        self.wheel.bottom_wheel_ccw = 0
        self.wheel.bottom_wheel_cw = 0
        self.wheel.left_wheel_ccw = 0
        self.wheel.left_wheel_cw = 0
        self.wheel.right_wheel_ccw = 0
        self.wheel.right_wheel_cw = 0

        self.navigationSetting = NavigationSetting()
        self.navigationSetting.direction = 0
        self.navigationSetting.rotation = 0
        self.navigationSetting.speed = 0

        self.robotStatus = RobotStatus()
        self.robotStatus.status = "stop"

        self.controlOutput = ControlOutput()
        self.controlOutput.outw = 0
        self.controlOutput.outy = 0

        self.setPoint = SetPoint()
        self.setPoint.distance = 0
        self.setPoint.angle = 0

        self.gripper = DribbleModule()
        self.coordinate = Coordinate()
        self.mode = RobotMode()
        self.dataBola = BallPositionBasedOnCamera()
        
        self.pidSpeed = PID(5, 0, 0, 0.1)
        self.pidAx = PID(8, 0, 0, 0.1)
        self.pidAy = PID(8, 0, 0, 0.1)
        self.pidW = PID(13, 0, 0, 0.1)
        self.pidAxBall = PID(5, 0, 0, 0.1)
        self.pidWBall = PID(10, 0, 0, 0.1)

        self.ball_distance = 0 # ada dua ball distance
        self.positionSetPoint = 0
        self.angleSetPoint = 0
        self.heading = 0
        self.flag = 0
        self.robotActionStatus = False
        self.chargingStatus = False
        self.robotMode = 'nothing'
        self.jalan = 0
        self.beforeSpeed = 0
        self.calculatePIDDistance = 0
        self.afterSpeed = 0
        self.headingSetPoint = -5
        self.ballAngle = 0
        self.errorX = 0
        self.errorY = 0
        self.errorHeading = 0
        self.batas = 3
        self.maxSpeed = 0.5
        self.maxPWM = 150
        self.subProses = 0
        self.stepKicker = 0
        self.currentTime = 0
        self.currentTimeRest = 0
        self.startKick = False
        self.restKicker = False
        self.firstKick = True
        self.counter = 0
        self.statusKicker = False
        self.kickFinish = False
        self.ballCoordinateX = 0
        self.ballCoordinateY = 0
        self.xBolaReal = 0
        self.yBolaReal = 0
        self.ballDistance = 0 # ada dua ball distance
        self.step = 0
        self.stepTarget = 0

    def navigationSettingCallback(self, msg: NavigationSetting):
        self.navigationSetting.speed = msg.speed
        self.navigationSetting.direction = msg.direction
        self.navigationSetting.rotation = msg.rotation
        if self.robotStatus.status != "stop" and self.robotStatus.status:
            self.navigateRobot()

    def robotStatusCallback(self, msg: RobotStatus):
        self.robotStatus.status = msg.status
        if self.robotStatus.status == "stop":
            self.stopRobot()
        elif self.robotStatus.status == "running":
            self.navigateRobot()

    def controlOutputCallback(self, msg: ControlOutput):
        self.pidAxBall = msg.outy
        self.pidWBall = msg.outw
        ay = 0

    def coordinateCallback(self, msg: Coordinate):
        self.coordinate = msg

    def headingCallback(self, msg: Float32):
        self.heading = msg.data

    def headingSetPointCallback(self, msg: Float32):
        self.headingSetPoint = msg.data

    def robotModeCallback(self, msg: RobotMode):
        self.robotMode = msg.mode
        self.robotActionStatus = True
    
    def ballPositionBasedOnCameraCallback(self, msg: BallPositionBasedOnCamera):
        self.positionSetPoint = msg.position_set_point
        self.angleSetPoint = msg.angle_set_point

    def ballCoordinateCallback(self, msg: PoseArray):
        if msg.poses is not None:  # Cek apakah ada data bola
            for i, pose in enumerate(msg.poses):
                self.ballCoordinateX = pose.position.x
                self.ballCoordinateY = pose.position.y
                conf = pose.position.z  # Confidence score
                # rospy.loginfo(f"Bola {i+1}: x={self.ballCoordinate_x}, y={self.ballCoordinate_y}, conf={conf}")

        x2 = CAMERA_MIDDLE_POINT["x"] - self.ballCoordinateX
        y2 = self.ballCoordinateY - CAMERA_MIDDLE_POINT["y"]
        r = math.sqrt(x2**2 + y2**2)
        
        if r < 183:
            r2 = 0.0034 * r**2 + 0.056 * r
        elif 183 <= r <= 210:
            r2 = 0.2062 * r**2 - 73.552 * r + 6703
        else:
            r2 = 5.2466 * r**2 - 2222 * r + 235656

        sdt = math.atan2(y2, x2)
        sudut = math.degrees(sdt) + 180

        a, b = 0, 50
        deltax = -math.cos(math.radians(sudut)) * r2
        deltay = math.sin(math.radians(sudut)) * r2

        self.xbola_real = deltax + b
        self.ybola_real = deltay + a
        self.ballDistance = r2
        self.ballDirection = sudut

        self.dataBola.angle = int(sudut)
        self.dataBola.distance = int(r2)
        self.dataBola.x_real = int(self.xbola_real)
        self.dataBola.y_real = int(self.ybola_real)
        self.dataBola.x_on_camera = int(self.ballCoordinateX)
        self.dataBola.y_on_camera = int(self.ballCoordinateY)
        self.dataBola.angle_set_point = int(self.angleSetPoint)
        self.dataBola.position_set_point = int(self.positionSetPoint)

        self.pubBallPosition.publish(self.dataBola)
    
    def stopRobot(self):
        self.wheel.left_wheel_ccw = 0
        self.wheel.left_wheel_cw = 0
        self.wheel.right_wheel_ccw = 0
        self.wheel.right_wheel_cw = 0
        self.wheel.bottom_wheel_ccw = 0
        self.wheel.bottom_wheel_cw = 0
        self.wheel.right_wheel_cw = 0

        self.pubPWM.publish(self.wheel)

def main(args=None):
    rclpy.init(args=args)
    node = RobotMovementAutoNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()