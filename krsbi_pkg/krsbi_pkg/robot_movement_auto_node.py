#!/usr/bin/env python3
import math
import time
import heapq
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float32, String
from krsbi_pkg.constants import CAMERA_MIDDLE_POINT, BALL_SET_POINT
from krsbi_interfaces.msg import Wheel, DribbleModule, NavigationSetting, RobotStatus, Coordinate, RobotMode, SetPoint, BallPositionBasedOnCamera, RealSenseT265, KickerModule

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

        self.get_logger().info("Node robot_movement_auto_node sudah dijalankan")

        self.create_timer(0.01, self.robotAction)
        self.create_timer(0.1, self.kicker_update)
        self.create_timer(0.1, self.shortpass_update)
        self.create_timer(0.1, self.shooting_update)


        # Publisher
        self.pubPWM = self.create_publisher(Wheel, 'Wheel', 10)
        self.pubGripper = self.create_publisher(DribbleModule, 'DribbleModule', 10)
        self.pubBallPosition = self.create_publisher(BallPositionBasedOnCamera, '/rosserial/BallPositionBasedOnCamera', 10)
        self.pubKickerModule = self.create_publisher(KickerModule, 'KickerModule', 10)

        # Subscriber
        self.create_subscription(RealSenseT265, '/t265/Odometry', self.realSenseT265Callback, 10)
        self.create_subscription(Coordinate, '/ui/Coordinate', self.coordinateCallback, 10)
        self.create_subscription(Float32, '/rosserial/BNO', self.headingCallback, 10)
        self.create_subscription(Float32, 'HeadingSetPoint', self.headingSetPointCallback, 10)
        self.create_subscription(RobotMode, '/ui/RobotMode', self.robotModeCallback, 10)
        self.create_subscription(PoseArray, '/yolov5/ball', self.ballCoordinateCallback, 10)

        self.wheel = Wheel()
        self.wheel.bottom_wheel_ccw = 0.0
        self.wheel.bottom_wheel_cw = 0.0
        self.wheel.left_wheel_ccw = 0.0
        self.wheel.left_wheel_cw = 0.0
        self.wheel.right_wheel_ccw = 0.0
        self.wheel.right_wheel_cw = 0.0

        self.navigationSetting = NavigationSetting()
        self.navigationSetting.direction = 0.0
        self.navigationSetting.rotation = 0.0
        self.navigationSetting.speed = 0.0

        self.robotStatus = RobotStatus()
        self.robotStatus.status = "stop"

        self.setPoint = SetPoint()
        self.setPoint.distance = 0.0
        self.setPoint.angle = 0.0

        self.coordinate = Coordinate()
        self.coordinate.x = 0.0
        self.coordinate.y = 0.0

        self.realSenseT265 = RealSenseT265()
        self.realSenseT265.x = 0.0
        self.realSenseT265.y = 0.0

        self.gripper = DribbleModule()
        self.mode = RobotMode()
        self.dataBola = BallPositionBasedOnCamera()
        self.kickerModule = KickerModule()
        

        self.pidSpeed = PID(5, 0, 0, 0.1)
        self.pidAx = PID(8, 0, 0, 0.1)
        self.pidAy = PID(8, 0, 0, 0.1)
        self.pidW = PID(13, 0, 0, 0.1)
        self.pidAxBall = PID(5, 0, 0, 0.1)
        self.pidWBall = PID(10, 0, 0, 0.1)

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
        self.headingSetPoint = 0
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
        self.ballDistance = 0
        self.step = 0
        self.stepTarget = 0

        self.kickerStatus = "idle"
        self.chargeStartTime = 0.0
        self.chargeDuration = 0.0
        self.lastKickTime = 0.0
        self.cooldownDuration = 5.0
        self.kickFreezeDuration = 0.5
        self.kickTime = 0.0

        self.shortpassStep = 0
        self.shortpassStartTime = 0.0
        self.shortpassActive = False

        self.shootingStep = 0
        self.shootingStartTime = 0.0
        self.shootingActive = False

    def realSenseT265Callback(self, msg: RealSenseT265):
        self.realSenseT265.x = msg.x
        self.realSenseT265.z = msg.z
        self.realSenseT265.y = msg.y
        self.realSenseT265.heading = msg.heading

    def coordinateCallback(self, msg: Coordinate):
        self.coordinate = msg

    def headingCallback(self, msg: Float32):
        self.heading = msg.data

    def headingSetPointCallback(self, msg: Float32):
        self.headingSetPoint = msg.data

    def robotModeCallback(self, msg: RobotMode):
        self.robotMode = msg.mode
        self.robotActionStatus = True
    
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
        self.dataBola.position_set_point = BALL_SET_POINT["distance"]
        self.dataBola.angle_set_point = BALL_SET_POINT["angle"]
        self.dataBola.position_x_set_point = BALL_SET_POINT["x_onCamera"]
        self.dataBola.position_y_set_point = BALL_SET_POINT["y_onCamera"]

        self.pubBallPosition.publish(self.dataBola)
    
    def calculateErrorAxAy(self, xNow, yNow, xTarget, yTarget):
        # Menghitung selisih koordinat
        deltaX = xTarget - xNow
        deltaY = yTarget - yNow
        
        return deltaX, deltaY
    
    def calculateHeadingError(self, heading, headingSetPoint) :
        # Normalize the angle to be within 0-360 degrees
        headingSetPoint = headingSetPoint % 360

        # Calculate the shortest path (clockwise or counterclockwise)
        heading_error = headingSetPoint - heading

        # Make sure the error is the shortest path (i.e., within -180 to 180 degrees)
        if heading_error > 180:
            heading_error -= 360
        elif heading_error < -180:
            heading_error += 360
        
        return heading_error
    
    def constrain(self, value, min_value, max_value):
        return max(min_value, min(max_value, value))
    
    def wheel_kinematics(self, outputAy, outputAx, outputW):
        force1 = ((-0.5773509529248335 * outputAy) + (-0.33333368867682667 * outputAx) + (0.33333368867682667 * outputW))
        force2 = ((0.5773509529248335 * outputAy) + (-0.3333329779898402 * outputAx) + (0.3333329779898402 * outputW))
        force3 = ((0 * outputAy) + (0.6666666666666669 * outputAx) + (0.33333333333333326 * outputW))
        # print(f"{force1} {force2} {force3}")

        # Normalisasi input
        fx = self.constrain(outputAy, -self.maxSpeed, self.maxSpeed)
        fy = self.constrain(outputAx, -self.maxSpeed, self.maxSpeed)
        fw = self.constrain(outputW, -self.maxSpeed, self.maxSpeed)

        # Kurangi sensitivitas jika ada kombinasi gerakan
        combined_factor = 0.7  # Faktor pengurang sensitivitas
        if fx != 0 and fy != 0:
            fx *= combined_factor
            fy *= combined_factor

        # Hitung kecepatan PWM berdasarkan kombinasi gerakan
        pwm_db1 = int(math.sqrt(fx ** 2 + fy ** 2) / self.maxSpeed * self.maxPWM)
        pwm_db2 = pwm_db1  # Sama untuk kedua roda

        if (force1 >= 0):
            f1CW = np.interp(force1, [0, 0.91], [0, 255])
            f1CCW = 0

        if (force1 < 0):
            force1 = abs(force1)
            f1CW = 0
            f1CCW = np.interp(force1, [0, 0.91], [0, 255])

        if (force2 >= 0):
            f2CW = np.interp(force2, [0, 0.91], [0, 255])
            f2CCW = 0

        if (force2 < 0):
            force2 = abs(force2)
            f2CW = 0
            f2CCW = np.interp(force2, [0, 0.91], [0, 255])

        if (force3 >= 0):
            f3CW = np.interp(force3, [0, 0.91], [0, 255])
            f3CCW = 0

        if (force3 < 0):
            force3 = abs(force3)
            f3CW = 0
            f3CCW = np.interp(force3, [0, 0.91], [0, 255])
        
        self.wheel.left_wheel_cw = float(f1CW)
        self.wheel.left_wheel_ccw = float(f1CCW)
        self.wheel.right_wheel_cw = float(f2CW)
        self.wheel.right_wheel_ccw = float(f2CCW)
        self.wheel.bottom_wheel_cw = float(f3CW)
        self.wheel.bottom_wheel_ccw = float(f3CCW)
        self.pubPWM.publish(self.wheel)
        #self.pubGripper.publish(self.gripper)

    def remap(self, value, old_min, old_max, new_min, new_max):
        """
        Memetakan suatu nilai dari satu rentang ke rentang lainnya.

        Parameters:
        - value (float): Nilai yang akan di-remap.
        - old_min (float): Batas bawah rentang asal.
        - old_max (float): Batas atas rentang asal.
        - new_min (float): Batas bawah rentang tujuan.
        - new_max (float): Batas atas rentang tujuan.

        Returns:
        - float: Nilai yang telah dipetakan ke rentang baru.
        """
        # Pastikan rentang asal tidak nol (untuk menghindari pembagian dengan nol)
        if old_max - old_min == 0:
            raise ValueError("Rentang asal tidak boleh nol.")
        
        # Rumus remap
        scaled = (value - old_min) / (old_max - old_min)
        remapped_value = new_min + (scaled * (new_max - new_min))
        return remapped_value
    
    def robotDirection(self, xTarget, yTarget, xNow, yNow) :
        deltaX = xTarget - xNow
        deltaY = yTarget - yNow

        return math.atan2(deltaY, deltaX) * 180 / math.pi
    
    def controlOutputRobotDirection(self, coordinateX, coordinateY) :
        robotTargetDirection = self.robotDirection(coordinateX, coordinateY, self.realSenseT265.x, self.realSenseT265.y)
        self.error_heading = self.calculateHeadingError(self.heading, robotTargetDirection) # Robot jalan sambil mengoreksi arah menuju titik target (robot menyesuaikan arah ke titik target)
        ax = 0
        ay = 0
        w = (self.pidW(self.error_heading))
        w = self.remap(w, 0, 1000, 0, 1)

        min_speed_w = 0.15
        max_speed_w = 0.5

        if w > max_speed_w:
            w = max_speed_w
        elif w < -max_speed_w:
            w = -max_speed_w

        if 0 < w < min_speed_w:
            w = min_speed_w
        elif -min_speed_w < w < 0:
            w = -min_speed_w

        # print(f"robotTargetDirection = {robotTargetDirection}\nheading = {self.heading}\nerror heading = {self.error_heading}\nw = {w}")
        
        self.wheel_kinematics(ax, ay, w)
        if -2 < self.error_heading < 2:
            self.wheel_kinematics(0, 0, 0)

    def controlOutputGoToCoordinate(self, coordinateX, coordinateY):
        error_x, error_y = self.calculateErrorAxAy(self.realSenseT265.x, self.realSenseT265.y, coordinateX, coordinateY)
        error_heading = self.calculateHeadingError(self.heading, self.headingSetPoint)   # Robot terus menghadap set point

        ax = (self.pidAx(error_x))
        ay = -(self.pidAy(error_y))
        w = (self.pidW(error_heading))

        # Remap
        ax = self.remap(ax, 0, 1000, 0, 1)
        ay = self.remap(ay, 0, 1000, 0, 1)
        w = self.remap(w, 0, 1000, 0, 1)

        max_speed = 0.8

        if ax > max_speed:
            ax = max_speed
        elif ax < -max_speed:
            ax = -max_speed
        
        if ay > max_speed:
            ay = max_speed
        elif ay < -max_speed:
            ay = -max_speed

        if w > max_speed:
            w = max_speed
        elif w < -max_speed:
            w = -max_speed
        
        # Tambahkan batas minimum kecepatan agar roda tidak terlalu lambat
        min_speed = 0.1  # Set batas minimum yang sesuai dengan sistem

        min_speed_w = 0.15

        # Cek jika kecepatan terlalu kecil, berikan nilai minimum yang cukup untuk menggerakkan roda
        if 0 < ax < min_speed:
            ax = min_speed
        elif -min_speed < ax < 0:
            ax = -min_speed

        if 0 < ay < min_speed:
            ay = min_speed
        elif -min_speed < ay < 0:
            ay = -min_speed

        if 0 < w < min_speed_w:
            w = min_speed_w
        elif -min_speed_w < w < 0:
            w = -min_speed_w

        # if (self.coordinate.y - 3) < (self.t265.y) < (self.coordinate.y + 3):
        #     ay = 0

        # if (self.coordinate.x - 3) < (self.t265.x) < (self.coordinate.x + 3):
        #     ax = 0

        # if (self.heading_setPoint - 1) < (self.heading) < (self.heading_setPoint + 1):
        #     w = 0
            
        # print(f"+ ax\t: {ax}\n+ ay\t: {ay}\n+ w\t: {w}")
        
        self.errorX = error_x
        self.errorY = error_y
        # self.error_heading = error_heading
    
        self.wheel_kinematics(ax, ay, w)

    def errorDirection(self, sudutSetPoint, sudut) :
        if (sudutSetPoint - sudut > 180): sudut += 360
        elif (sudut - sudutSetPoint > 180): sudut -= 360
        return sudutSetPoint - sudut

    def catchBall(self):
        errorJarak = BALL_SET_POINT["distance"] - self.ballDistance
        errorSudut = self.errorDirection(BALL_SET_POINT["angle"], self.ballDirection)
        if -self.batas < errorJarak < self.batas and -self.batas < errorSudut < self.batas:
            return True
        else:
            return False
    
    def catchBall2(self):
        error_jarak = BALL_SET_POINT["distance"] - self.ballDistance
        error_sudut = BALL_SET_POINT["angle"] - self.ballDirection
        if -self.batas < error_jarak < self.batas and -self.batas < error_sudut < self.batas:
            return True
        else:
            return False
        
    def passCharging(self):
        if self.kickerStatus != "idle":
            self.get_logger().info(f"Tidak bisa charging pass, status: {self.kickerStatus}")
            return
         
        self.kickerStatus = "charging_pass"
        self.chargeDuration = 3.0
        self.chargeStartTime = time.time()
        self.get_logger().info("Mulai charging PASS")

    def shootCharging(self):
        if self.kickerStatus != "idle":
            self.get_logger().info(f"Tidak bisa charging shoot, status: {self.kickerStatus}")
            return
        self.kickerStatus = "charging_shoot"
        self.chargeDuration = 7.0
        self.chargeStartTime = time.time()
        self.get_logger().info("Mulai charging SHOOT")

    def kick(self):
        if self.kickerStatus != "ready_to_kick":
            self.get_logger().info(f"Tidak bisa kick, status: {self.kickerStatus}")
            return

        self.get_logger().info("KICK!")
        self.kickerModule.status = "kick"
        self.pubKickerModule.publish(self.kickerModule)

        self.kickerStatus = "kick_freeze"  # status baru
        self.kickTime = time.time()

        self.wheel_kinematics(0,0,0)

    def shortPass(self):
        if self.shortpassActive or self.kickerStatus != "idle":
            # self.get_logger().info("Short pass sedang aktif atau kicker belum siap.")
            return

        self.get_logger().info("Memulai short pass...")
        self.shortpassStep = 1
        self.shortpassStartTime = time.time()
        self.shortpassActive = True

    def shooting(self):
        if self.shootingActive or self.kickerStatus != "idle":
            # self.get_logger().info("Short pass sedang aktif atau kicker belum siap.")
            return

        self.get_logger().info("Memulai shooting...")
        self.shootingStep = 1
        self.shootingStartTime = time.time()
        self.shootingActive = True

    def kicker_update(self):
        now = time.time()

        if self.kickerStatus in ["charging_pass", "charging_shoot"]:
            if now - self.chargeStartTime < self.chargeDuration:
                self.kickerModule.status = "charging"
                self.pubKickerModule.publish(self.kickerModule)
            else:
                self.kickerStatus = "ready_to_kick"
                self.kickerModule.status = "ready_to_kick"
                self.pubKickerModule.publish(self.kickerModule)
                self.get_logger().info("Selesai charging. Siap untuk kick.")

        elif self.kickerStatus == "kick_freeze":
            if now - self.kickTime >= self.kickFreezeDuration:
                self.kickerStatus = "cooldown"
                self.lastKickTime = now
                self.get_logger().info("Freeze selesai. Mulai cooldown.")

        elif self.kickerStatus == "cooldown":
            if now - self.lastKickTime >= self.cooldownDuration:
                self.kickerStatus = "idle"
                self.get_logger().info("Cooldown selesai. Siap charging lagi.")
                self.kickerModule.status = "finish"
                self.pubKickerModule.publish(self.kickerModule)

    def shortpass_update(self):
        if not self.shortpassActive:
            return

        now = time.time()
        elapsed = now - self.shortpassStartTime
        print(elapsed)

        if self.shortpassStep == 1:
            if elapsed < 3.0:
                self.kickerModule.status = "charging"
                self.pubKickerModule.publish(self.kickerModule)
            else:
                self.get_logger().info("Charging selesai. KICK!")
                self.kickerModule.status = "kick"
                self.pubKickerModule.publish(self.kickerModule)
                self.shortpassStep = 2
                self.shortpassStartTime = now  # reset timer

        elif self.shortpassStep == 2:
            if elapsed >= 0.5:
                self.get_logger().info("Freeze selesai. Mulai cooldown.")
                self.shortpassStep = 3
                self.shortpassStartTime = now

        elif self.shortpassStep == 3:
            if elapsed >= 5.0:
                self.get_logger().info("Cooldown selesai. Short pass selesai.")
                self.kickerStatus = "idle"
                self.shortpassActive = False
                self.shortpassStep = 0
                self.kickerModule.status = "finish"
                self.pubKickerModule.publish(self.kickerModule)
                self.robotMode = "nothing"

    def shooting_update(self):
        if not self.shootingActive:
            return

        now = time.time()
        elapsed = now - self.shootingStartTime
        print(elapsed)

        if self.shootingStep == 1:
            if elapsed < 7.0:
                self.kickerModule.status = "charging"
                self.pubKickerModule.publish(self.kickerModule)
            else:
                self.get_logger().info("Charging selesai. KICK!")
                self.kickerModule.status = "kick"
                self.pubKickerModule.publish(self.kickerModule)
                self.shootingStep = 2
                self.shootingStartTime = now  # reset timer

        elif self.shootingStep == 2:
            if elapsed >= 0.5:
                self.get_logger().info("Freeze selesai. Mulai cooldown.")
                self.shootingStep = 3
                self.shootingStartTime = now

        elif self.shootingStep == 3:
            if elapsed >= 5.0:
                self.get_logger().info("Cooldown selesai. Shooting selesai.")
                self.kickerStatus = "idle"
                self.shootingActive = False
                self.shootingStep = 0
                self.kickerModule.status = "finish"
                self.pubKickerModule.publish(self.kickerModule)
                self.robotMode = "nothing"

    def robotAction(self):
        if self.robotMode == "nothing":
            if self.robotActionStatus:
                print("Robot Mode: Nothing")
                self.robotActionStatus = False
            self.flag = 0   
            self.wheel_kinematics(0,0,0)
            self.kickFinish = False
            self.stepKicker = 0

        if self.robotMode == "goHome":
            if self.robotActionStatus:
                print("Robot Mode: Go Home")
                self.robotActionStatus = False
            self.controlOutputGoToCoordinate(0, 0)
            if -self.batas <= self.errorX <= self.batas and -self.batas <= self.errorY <= self.batas:
                self.robotMode = "nothing"

        if self.robotMode == "goToCoordinate":
            x = self.coordinate.x
            y = self.coordinate.y
            if self.robotActionStatus:
                print(f"Robot Mode: Go To Coordinate {x, y}")
                self.robotActionStatus = False
            self.controlOutputGoToCoordinate(x, y)
            if -self.batas <= self.errorX <= self.batas and -self.batas <= self.errorY <= self.batas : 
                self.robotMode = 'nothing'

        if self.robotMode == "catchBall":
            state = self.catchBall()
            if state:
                self.robotMode = "nothing"

        if self.robotMode == "catchBall2":
            state = self.catchBall2()
            if state:
                self.robotMode = "nothing"

        if self.robotMode == "passing":
            self.shortPass()

        if self.robotMode == "shooting":
            self.shooting()


def main(args=None):
    rclpy.init(args=args)
    node = RobotMovementAutoNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()