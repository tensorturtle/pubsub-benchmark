import os 
import sys
from time import sleep

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int8

#import cv2
import numpy as np

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        
        self.test_type = os.environ.get('TEST_TYPE')

        self.cv_bridge = CvBridge()

        if self.test_type is None:
            self.get_logger().warning('TEST_TYPE not set. Defaulting to "small"')
            self.test_type = 'small'

        # t1_topic is the first topic in the chain. Repeater nodes create more topics
        # tf_topic is the final topic, which bypasses the repeaters as a control

        if self.test_type == 'small':
            self.publisher_t0 = self.create_publisher(Int8, 't0_topic', 0)
            self.publisher_t1 = self.create_publisher(Int8, 't1_topic', 0)
        elif self.test_type == 'medium':
            self.publisher_t0 = self.create_publisher(Int8, 't0_topic', 0)
            self.publisher_t1 = self.create_publisher(Int8, 't1_topic', 0)
        elif self.test_type == 'large':
            self.publisher_t0 = self.create_publisher(Image, 't0_topic', 0)
            self.publisher_t1 = self.create_publisher(Image, 't1_topic', 0)


        timer_period = 2 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 1

    def timer_callback(self):
        if self.i == 0:
            #self.timer.stop()
            #self.timer.cancel()
            #self.timer.destroy()
            #self.publisher_destroy(self.publisher_t0)
            #self.publisher_destroy(self.publisher_t1)
            self.destroy_node()
            sys.exit(0)

        self.i += 1
        self.i = self.i % 20

        if self.test_type == 'small':
            msg = Int8()
            msg.data = self.i
            self.publisher_t0.publish(msg)
            self.publisher_t1.publish(msg)
            msg_repr = f'{msg.data}'

        elif self.test_type == 'medium':
            msg = Int8()
            msg.data = self.i
            self.publisher_t0.publish(msg)
            self.publisher_t1.publish(msg)
            msg_repr = f'{msg.data}'
        elif self.test_type == 'large':

            # 'zero-like' is a kill signal to following nodes
            if self.i == 0:
                img = np.zeros((640,480, 3), dtype=np.uint8)
            else:
                img = np.random.randint(0, 254, (640,480, 3), dtype=np.uint8) + 1 # avoid accidentally sending a zero-like

            msg = self.cv_bridge.cv2_to_imgmsg(img, encoding='bgr8')
            self.publisher_t0.publish(msg)
            self.publisher_t1.publish(msg)
            msg_repr = f'Image with height: {msg.height}, width: {msg.width}'

        self.get_logger().info('Publishing: "%s"' % msg_repr)




def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()