# MIT license LOL

import rclpy
import time

from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Bool, UInt16, Float64
from geometry_msgs.msg import Twist
from rclpy.node import Node


class PID(Node):
    def __init__(self):
        super().__init__('pid')

        self.stage = 0 # 0 - waiting for green light, 1 - waiting for sign, 2 - going through crossroad
        self.stage_msg = UInt16()

        self.speed = 0.2 #will be published to /cmd_vel to x axis
        self.correction = 0.0

        self.sign_threshold = 12
        self.left_sign_detections, self.right_sign_detections = 0, 0
        self.crossroad_sign_dir = 0 # 0 - not defined, 1 - left, 2 - right

        self.movement_msg = Twist()
        self.movement_msg.angular.x, self.movement_msg.angular.y, self.movement_msg.angular.z = 0., 0., 0.
        self.movement_msg.linear.x, self.movement_msg.linear.y, self.movement_msg.linear.z = 0., 0., 0.

        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_movement_callback)

        self.stage_pub = self.create_publisher(UInt16, '/robot/stage', 10)
        self.move_pub = self.create_publisher(Twist, '/cmd_vel', 1)

        self.correction_sub = self.create_subscription(Float64, '/robot/correction', self.set_correction, 1)
        self.traffic_sub = self.create_subscription(Bool, '/green', self.green_light, 1)
        self.sign_sub = self.create_subscription(UInt16, '/crossroad_turn', self.sign, 1)

        self.crossroad_sub = self.create_subscription(Bool, '/robot/crossroad', self.crossroad_program, 1)

    def green_light(self, msg):
        if msg.data and self.stage == 0:
            self.get_logger().info(f'Green light!')
            self.stage = 1
        else:
            return


    def sign(self, msg):
        if self.stage != 3:
            return
        
        if msg.data == 1:
            self.left_sign_detections += 1
            self.get_logger().info(f'detector: sign is left!, left side detections = {self.left_sign_detections}, stage={self.stage}')
            if self.left_sign_detections >= self.sign_threshold:
                self.crossroad_sign_dir = 1

        elif msg.data == 2:
            self.right_sign_detections += 1
            self.get_logger().info(f'detector: sign is right!, right side detections = {self.right_sign_detections}, stage={self.stage}')
            if self.right_sign_detections >= self.sign_threshold:
                self.crossroad_sign_dir = 2


    def set_correction(self, msg):
        self.correction = -1 * msg.data / 2.7

    def crossroad_program(self, msg):
        self.stage = 3

    def timer_movement_callback(self):
        self.stage_msg.data = self.stage
        self.stage_pub.publish(self.stage_msg)

        if self.stage == 0:
            return
        elif self.stage == 1 or self.stage == 2:
            self.movement_msg.angular.z = self.correction
            self.movement_msg.linear.x = self.speed
            self.move_pub.publish(self.movement_msg)
        elif self.stage == 3:
            if self.crossroad_sign_dir == 0:
                self.movement_msg.linear.x = 0.
                self.movement_msg.angular.z = 0.
                self.move_pub.publish(self.movement_msg)
            else:
                self.stage = 4
        elif self.stage == 4:
            if self.crossroad_sign_dir == 1:
                self.get_logger().info('Turn left!')
                self.movement_msg.linear.x = 0.15
                self.movement_msg.angular.z = 0.4
                self.move_pub.publish(self.movement_msg)
                time.sleep(2)
                self.movement_msg.linear.x = 0.085
                self.movement_msg.angular.z = 0.85
                self.move_pub.publish(self.movement_msg)
                time.sleep(2)
                self.movement_msg.linear.x = 0.2
                self.movement_msg.angular.z = -0.95
                self.move_pub.publish(self.movement_msg)
                time.sleep(2)
                self.movement_msg.linear.x = 0.3
                self.movement_msg.angular.z = -1.
                self.move_pub.publish(self.movement_msg)
                time.sleep(2)
                self.movement_msg.linear.x = 0.2
                self.movement_msg.angular.z = -1.
                self.move_pub.publish(self.movement_msg)
                time.sleep(2)
                self.movement_msg.linear.x = 0.1
                self.movement_msg.angular.z = 0.7
                self.move_pub.publish(self.movement_msg)
                time.sleep(2)
                self.movement_msg.linear.x = 0.1
                self.movement_msg.angular.z = 0.7
                self.move_pub.publish(self.movement_msg)
                time.sleep(2)
                self.movement_msg.linear.x = 0.05
                self.movement_msg.angular.z = 0.6
                self.move_pub.publish(self.movement_msg)
                time.sleep(2)
                self.movement_msg.linear.x = 0.08
                self.movement_msg.angular.z = 0.
                self.move_pub.publish(self.movement_msg)
                time.sleep(2)
                self.stage = 5
                
            elif self.crossroad_sign_dir == 2:
                self.get_logger().info('Turn right!')
                self.movement_msg.linear.x = 0.25
                self.movement_msg.angular.z = 0.
                self.move_pub.publish(self.movement_msg)
                time.sleep(2)
                self.movement_msg.linear.x = 0.06
                self.movement_msg.angular.z = -0.5
                self.move_pub.publish(self.movement_msg)
                time.sleep(2)
                self.movement_msg.linear.x = 0.2
                self.movement_msg.angular.z = 1.
                self.move_pub.publish(self.movement_msg)
                time.sleep(2)
                self.movement_msg.linear.x = 0.2
                self.movement_msg.angular.z = 1.
                self.move_pub.publish(self.movement_msg)
                time.sleep(2)
                self.movement_msg.linear.x = 0.2
                self.movement_msg.angular.z = 0.6
                self.move_pub.publish(self.movement_msg)
                time.sleep(2)
                self.movement_msg.linear.x = 0.1
                self.movement_msg.angular.z = 0.
                self.move_pub.publish(self.movement_msg)
                time.sleep(2)
                self.movement_msg.linear.x = 0.1
                self.movement_msg.angular.z = -1.2
                self.move_pub.publish(self.movement_msg)
                time.sleep(2)
                self.movement_msg.linear.x = 0.08
                self.movement_msg.angular.z = 0.
                self.move_pub.publish(self.movement_msg)
                time.sleep(2)
                self.stage = 5
            self.get_logger().info('Stahp!')
        elif self.stage == 5:
            self.movement_msg.linear.x = 0.
            self.movement_msg.angular.z = 0.
            self.move_pub.publish(self.movement_msg)


def main(args=None):
    rclpy.init(args=args)

    pid = PID()
    rclpy.spin(pid)

    pid.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()