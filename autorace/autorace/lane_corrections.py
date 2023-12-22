# MIT license LOL

import rclpy
import cv2, os
import numpy as np

from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Bool, UInt16, Float64
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge

#this class publishes corrections [-1; 1] to angle, where -1 - utmost left and 1 - utmost right correction
class Corrector(Node):
    def __init__(self):
        super().__init__('corrector')

        self.img_sub = self.create_subscription(Image, '/color/image', self.image_callback, 1)
        self.corrections_pub = self.create_publisher(Float64, '/robot/correction', 10)
        self.crossroad_pub = self.create_publisher(Bool, '/robot/crossroad', 10)

        self.corrections = Float64()
        self.stage_publicable = UInt16()

        self.cv_bridge = CvBridge()
        self.stage = UInt16()
        self.stage.data = 0

        self.got_init_frame = False
        self.mask = None
        self.default_yellow_share, self.default_white_share, self.mask_general_pixels = 0, 0, 0
        self.prev_angular = 0.
        self.turns = 0
        self.activated_3_stage = False
        self.is_turning = False
        self.cumsum = 0.


    def image_callback(self, img_msg):
        img = self.cv_bridge.imgmsg_to_cv2(img_msg, 'bgr8')[250:400, 200:600]
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        if not self.got_init_frame:
            self.got_init_frame = True

            #set mask and default percentage
            mask_yellow = cv2.inRange(img, (20, 100, 100), (30, 255, 255))
            mask_white = cv2.inRange(img, (0,0,245), (172,111,255))
            self.mask = np.add(mask_yellow, mask_white)

            self.mask_general_pixels = cv2.countNonZero(self.mask)
            y = cv2.countNonZero(mask_yellow)
            w = cv2.countNonZero(mask_white)
            self.default_yellow_share, self.default_white_share = round(y/self.mask_general_pixels, 3), round(w/self.mask_general_pixels, 3)

        img_masked = cv2.bitwise_and(img, img, mask=self.mask)

        yellow_masked = cv2.countNonZero(cv2.inRange(img_masked, (20, 100, 100), (30, 255, 255)))
        white_masked = cv2.countNonZero(cv2.inRange(img_masked, (0,0,245), (172,111,255)))
        a_masked = yellow_masked + white_masked

        diff_y = -1 * (self.default_yellow_share - round(yellow_masked/(a_masked+0.00001), 3)) / self.default_yellow_share # differentials normalised
        diff_w = (self.default_white_share - round(white_masked/(a_masked+0.00001), 3)) / self.default_white_share

        self.corrections.data = diff_y + diff_w if self.cumsum > -150. else (diff_y + diff_w)/1.5

        if self.cumsum <= -200. and not self.activated_3_stage:
            crossroad_msg = Bool()
            crossroad_msg.data = True
            self.crossroad_pub.publish(crossroad_msg)
            self.activated_3_stage = True

        self.cumsum += self.corrections.data
        # self.get_logger().info(f'Cumsum: {self.cumsum}')
        self.corrections_pub.publish(self.corrections)



def main(args=None):
    rclpy.init(args=args)
    corrector = Corrector()
    rclpy.spin(corrector)
	
    corrector.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()