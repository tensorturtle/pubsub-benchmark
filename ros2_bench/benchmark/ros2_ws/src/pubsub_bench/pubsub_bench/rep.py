import os 
import sys
from time import sleep
import logging

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int8

import numpy as np

class Repeater(Node):
    def __init__(self, name, listen_to_topic, publish_to_topic, test_type):
        super().__init__(name)
        self.listen_to_topic = listen_to_topic
        self.publish_to_topic = publish_to_topic
        self.test_type = test_type

        self.cv_bridge = CvBridge()

        if self.test_type == 'small':
            self.subscription = self.create_subscription(Int8, listen_to_topic, self.listener_callback, 0)
            self.publisher = self.create_publisher(Int8, publish_to_topic, 0)
        
        elif self.test_type == 'medium':
            self.subscription = self.create_subscription(Int8, listen_to_topic, self.listener_callback, 0)
            self.publisher = self.create_publisher(Int8, publish_to_topic, 0)

        elif self.test_type == 'large':
            self.subscription = self.create_subscription(Image, listen_to_topic, self.listener_callback, 0)
            self.publisher = self.create_publisher(Image, publish_to_topic, 0)

        self.msg_counter = 0
        
    def listener_callback(self, msg, recode_msg=True):
        #self.get_logger().info(f'I heard: {msg.data} on {self.listen_to_topic}, bouncing it to {self.publish_to_topic}')
        msg_repr = str(msg.data)[:100]
        self.get_logger().info(f'I heard: {msg_repr} on {self.listen_to_topic}, bouncing it to {self.publish_to_topic}')
        #logging.info(f'I heard: {msg_repr} on {self.listen_to_topic}, bouncing it to {self.publish_to_topic}')

        if self.test_type == 'small':
            # if zero-like, send that kill signal down the line, and kill this node
            self.publisher.publish(msg)
            if msg.data == 0:
                self.get_logger().warning(f"Since {msg_repr} is zero-like, committing suicide")
                self.destroy_node()
                sys.exit(0)
        elif self.test_type == 'medium':
            self.publisher.publish(msg)
        elif self.test_type == 'large':
            if recode_msg:
                np_msg = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                msg = self.cv_bridge.cv2_to_imgmsg(np_msg, encoding='passthrough')

            # if zero-like, send that kill signal down the line, and kill this node
            self.publisher.publish(msg)
            if np.sum(np.array(msg.data)) == 0:
                self.get_logger().warning(f"Since {msg_repr} is zero-like, committing suicide")
                self.destroy_node()
                sys.exit(0)

def main(args=None):
    rclpy.init(args=args)

    test_type = os.environ.get('TEST_TYPE')

    # check if environmental variable NODE_COUNTER is set
    # if not, set it to 1
    node_counter = os.environ.get('NODE_COUNTER')
    if node_counter is None:
        node_counter = 1
    else:
        node_counter = int(node_counter)

    if test_type is None:
        logging.warning('TEST_TYPE not set. Defaulting to "small"')
        test_type = 'small'

    repeater = Repeater(f'repeater_{node_counter}', f't{node_counter}_topic', f't{node_counter+1}_topic', test_type)

    rclpy.spin(repeater)

    repeater.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()