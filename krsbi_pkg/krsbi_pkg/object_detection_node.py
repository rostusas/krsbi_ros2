#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import torch
import cv2
import time
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
from krsbi_interfaces.msg import BallPositionBasedOnCamera
from krsbi_pkg.constants import BALL_SET_POINT

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        # Publisher
        self.image_pub = self.create_publisher(Image, '/yolov5/detections', 10)
        self.ball_pub = self.create_publisher(PoseArray, '/yolov5/ball', 10)
        self.dummy_pub = self.create_publisher(PoseArray, '/yolov5/dummy', 10)
        self.ball_position_pub = self.create_publisher(BallPositionBasedOnCamera, '/rosserial/BallPosition', 10)

        # Subscriber
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)

        # Variable
        self.bridge = CvBridge()
        self.prev_time = time.time()

        # Model
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = torch.hub.load(
            '/home/rostu/yolov5', 
            'custom', 
            path='/home/rostu/krsbi_ws/src/krsbi_pkg/krsbi_pkg/KRSBI_55n.pt',
            source='local', 
            trust_repo=True
        ).to(device)

    def image_callback(self, msg):
        # Konversi ROS Image ke OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = cv2.flip(frame, 1)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        with torch.amp.autocast('cuda'):
            results = self.model(frame_rgb)

        frame = results.render()[0].copy()

        dataBola = BallPositionBasedOnCamera()
        ball_array = PoseArray()
        dummy_array = PoseArray()
        now = self.get_clock().now().to_msg()
        ball_array.header.stamp = now
        dummy_array.header.stamp = now
        ball_array.header.frame_id = "camera"
        dummy_array.header.frame_id = "camera"

        dataBola.position_set_point = BALL_SET_POINT["distance"]
        dataBola.angle_set_point = BALL_SET_POINT["angle"]
        dataBola.position_x_set_point = BALL_SET_POINT["x_onCamera"]
        dataBola.position_y_set_point = BALL_SET_POINT["y_onCamera"]

        for *xyxy, conf, cls in results.xyxy[0].cpu().numpy():
            x_min, y_min, x_max, y_max = map(int, xyxy)
            x_center = (x_min + x_max) / 2
            y_center = (y_min + y_max) / 2

            pose = Pose()
            pose.position.x = x_center
            pose.position.y = y_center
            pose.position.z = float(conf)

            if cls == 0:
                ball_array.poses.append(pose)
            elif cls == 1:
                dummy_array.poses.append(pose)

        self.ball_pub.publish(ball_array)
        self.dummy_pub.publish(dummy_array)
        self.ball_position_pub.publish(dataBola)

        # Publish hasil anotasi ke topic image
        annotated_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_pub.publish(annotated_image_msg)

        # FPS
        curr_time = time.time()
        fps = 1 / (curr_time - self.prev_time)
        self.prev_time = curr_time
        self.get_logger().info(f"FPS: {fps:.2f}")

        # Tambahkan ini untuk menampilkan video
        # cv2.imshow("YOLOv5 Detection", frame)
        # cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()