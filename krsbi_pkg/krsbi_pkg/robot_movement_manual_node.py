#!/usr/bin/env python3
import math
import time
import heapq
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32, String
from krsbi_interfaces.msg import Wheel, DribbleModule, DribbleSetting, NavigationSetting, RobotStatus

class RobotMovementManualNode(Node):
    def __init__(self):
        super().__init__("robot_movement_manual_node")

        self.get_logger().info("Node robot_movement_manual_node sudah dijalankan")


        # Publisher
        self.pubWheel = self.create_publisher(Wheel, 'Wheel', 10)
        self.pubDribbleModule = self.create_publisher(DribbleModule, 'DribbleModule', 10)

        # Subscriber
        self.create_subscription(DribbleSetting, '/ui/DribbleSetting', self.dribbleSettingCallback, 10)
        self.create_subscription(NavigationSetting, '/ui/NavigationSetting', self.navigationSettingCallback, 10)
        self.create_subscription(RobotStatus, '/ui/RobotStatus', self.robotStatusCallback, 10)

        self.wheel = Wheel()
        self.wheel.bottom_wheel_ccw = 0.0
        self.wheel.bottom_wheel_cw = 0.0
        self.wheel.left_wheel_ccw = 0.0
        self.wheel.left_wheel_cw = 0.0
        self.wheel.right_wheel_ccw = 0.0
        self.wheel.right_wheel_cw = 0.0

        self.navigationSetting = NavigationSetting()
        self.navigationSetting.speed = 0.0
        self.navigationSetting.direction = 0.0
        self.navigationSetting.rotation = 0.0

        self.dribbleModule = DribbleModule()
        self.dribbleModule.left_motor_cw = 0.0
        self.dribbleModule.left_motor_ccw = 0.0
        self.dribbleModule.right_motor_cw = 0.0
        self.dribbleModule.right_motor_ccw = 0.0

        self.robotStatus = RobotStatus()
        self.robotStatus.status = 'stop'

        
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

    def dribbleSettingCallback(self, msg: DribbleSetting):
        if msg.status == "stop":
            self.dribbleModule.left_motor_cw = msg.speed
            self.dribbleModule.left_motor_ccw = msg.speed
            self.dribbleModule.right_motor_cw = msg.speed
            self.dribbleModule.right_motor_ccw = msg.speed
        elif msg.status == "catch":
            self.dribbleModule.left_motor_cw = 0
            self.dribbleModule.left_motor_ccw = msg.speed
            self.dribbleModule.right_motor_cw = msg.speed
            self.dribbleModule.right_motor_ccw = 0

    def navigateRobot(self):
        degree, speed, rotation = self.navigationSetting.direction, self.navigationSetting.speed, self.navigationSetting.rotation
        theta1, theta2, theta3 = 150.0, 270.0, 30.0
        left_wheel = math.sin((degree - theta1) * (math.pi / 180.0))
        bottom_wheel = math.sin((degree - theta2) * (math.pi / 180.0))
        right_wheel = math.sin((degree - theta3) * (math.pi / 180.0))

        left_wheel = left_wheel * speed * (0 if (degree == 150.0 or degree == 330.0) else 1)
        bottom_wheel = bottom_wheel * speed * (0 if (degree == 90.0 or degree == 270.0) else 1)
        right_wheel = right_wheel * speed * (0 if (degree == 30.0 or degree == 210.0) else 1)

        wheel = [left_wheel,bottom_wheel,right_wheel]
        left_wheel,bottom_wheel,right_wheel = self.checkRotationCriteria(rotation,wheel,speed)

        left_wheel_cw = math.floor(abs(left_wheel))  if (left_wheel <= 0 )else 0
        left_wheel_ccw = math.floor(abs(left_wheel))   if (left_wheel >= 0)  else 0
        bottom_wheel_cw = math.floor(abs(bottom_wheel))   if (bottom_wheel <= 0) else 0
        bottom_wheel_ccw = math.floor(abs(bottom_wheel))   if (bottom_wheel >= 0 )else 0
        right_wheel_ccw = math.floor(abs(right_wheel))   if (right_wheel <= 0) else 0
        right_wheel_cw = math.floor(abs(right_wheel))   if (right_wheel >= 0 )else 0
        
        self.wheel.left_wheel_ccw = float(left_wheel_cw)
        self.wheel.left_wheel_cw = float(left_wheel_ccw)
        self.wheel.right_wheel_ccw = float(right_wheel_ccw)
        self.wheel.right_wheel_cw = float(right_wheel_cw)
        self.wheel.bottom_wheel_ccw = float(bottom_wheel_cw)
        self.wheel.bottom_wheel_cw = float(bottom_wheel_ccw)

        self.pubWheel.publish(self.wheel)

    def checkRotationCriteria(self, rotation, wheel, speed):

        if(speed == 0):
            wheel = [rotation,rotation,rotation]
            return wheel
        for i,value in enumerate(wheel):
            if (value*rotation >= 0 or (value == rotation)):
                if(sorted([a for a in wheel if a*rotation >= 0 ]).index(value) == 0):
                    wheel[i] = (value + rotation) if (value+rotation) <=255 and (value+rotation)>=-255 else 255 if value >=0 else -255
                    break
        return wheel

    def stopRobot(self):
        self.wheel.left_wheel_ccw = 0.0
        self.wheel.left_wheel_cw = 0.0
        self.wheel.right_wheel_ccw = 0.0
        self.wheel.right_wheel_cw = 0.0
        self.wheel.bottom_wheel_ccw = 0.0
        self.wheel.bottom_wheel_cw = 0.0

        self.pubWheel.publish(self.wheel)

def main(args=None):
    rclpy.init(args=args)
    node = RobotMovementManualNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()