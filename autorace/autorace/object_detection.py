# MIT license LOL

import rclpy
import cv2, os

from ament_index_python.packages import get_package_share_directory
from .submodules.detect import detect_green, detect_sign

from std_msgs.msg import Bool, UInt16
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge

class Detector(Node):

    def __init__(self):
        super().__init__('robot_app')
        self.light_pub = self.create_publisher(Bool, '/green', 10)
        self.turn_direction_pub = self.create_publisher(UInt16, '/crossroad_turn', 10)
        self.img_sub = self.create_subscription(Image, '/color/image', self.image_callback, 1)
        self.stage_sub = self.create_subscription(UInt16, '/robot/stage', self.set_stage, 1)

        self.cv_bridge = CvBridge()
        
        self.msg = Bool()
        self.sign_msg = UInt16()
        self.stage = UInt16()
        self.stage.data = 0 
        # 0 - waiting for the traffic light
        # 1 - finding and deciding turn sign
        # 2 - turn decided

        right_turn_texture_path = os.path.join(get_package_share_directory('autorace'), 'textures', 'traffic_right.png')
        left_turn_texture_path = os.path.join(get_package_share_directory('autorace'), 'textures', 'traffic_left.png')

        self.sign_right_tex, self.sign_left_tex = cv2.imread(right_turn_texture_path), cv2.imread(left_turn_texture_path)

    def image_callback(self, img_msg):
        image = self.cv_bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        if self.stage.data == 0:
            self.msg.data = detect_green(image)
            self.light_pub.publish(self.msg)
        elif self.stage.data >= 1:
            image = image[:, 0:350]
            cv2.imwrite('/home/yoy/Pictures/imag67.png', image)
            self.sign_msg.data = detect_sign(image, self.sign_left_tex, self.sign_right_tex)
            self.turn_direction_pub.publish(self.sign_msg)
        else:
            return
        
    def set_stage(self, new_stage):
        self.stage = new_stage

def main(args=None):
    rclpy.init(args=args)

    detector = Detector()
    rclpy.spin(detector)

    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()