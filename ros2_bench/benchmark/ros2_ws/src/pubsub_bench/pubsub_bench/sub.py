import os 
import sys
import time
import logging
from time import sleep

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int8

import numpy as np

s_to_ms = lambda x: round(x * 1000, 3)

class FinalSubscriber(Node):
    '''
    Compares the time difference between 
        1. Message sent directly from root publisher
        2. Message that has been bounced through all the repeaters
    '''
    def __init__(self, test_type, direct_topic='t0_topic', bounced_topic='t11_topic'):
        super().__init__('final_subscriber')
        self.test_type = test_type
        self.direct_topic = direct_topic
        self.bounced_topic = bounced_topic

        self.direct_received_time = 0
        self.bounced_received_time = 0
        self.time_deltas = []

        if self.test_type == 'small':
            self.subscription_1 = self.create_subscription(Int8, direct_topic, self.callback_direct, 0)
            self.subscription_f = self.create_subscription(Int8, bounced_topic, self.callback_bounced, 0)
        elif self.test_type == 'medium':
            self.subscription_1 = self.create_subscription(Int8, direct_topic, self.callback_direct, 0)
            self.subscription_f = self.create_subscription(Int8, bounced_topic, self.callback_bounced, 0)
        elif self.test_type == 'large':
            self.subscription_1 = self.create_subscription(Image, direct_topic, self.callback_direct, 0)
            self.subscription_f = self.create_subscription(Image, bounced_topic, self.callback_bounced, 0)
    
    def callback_direct(self, msg):
        self.direct_received_time = time.perf_counter()
    
    def callback_bounced(self, msg):
        self.bounced_received_time = time.perf_counter()
        self.time_deltas.append(self.bounced_received_time - self.direct_received_time)
        self.get_logger().info(
            f"Comparing received time between {self.direct_topic} and {self.bounced_topic}: {s_to_ms(self.time_deltas[-1])} ms"
        )
        if msg.data == 0 or np.sum(np.array(msg.data)) == 0: 
            self.get_logger().warning(f"Received message is zero-like. Commiting suicide.")
            sleep(1) # wait in case repeaters are not done
            self.get_logger().info(f"Average time delta: {s_to_ms(np.mean(self.time_deltas))} ms")
            self.get_logger().info(f"Std time delta: {s_to_ms(np.std(self.time_deltas))} ms")
            self.destroy_node()
            sys.exit(0)
            #TODO save to database



def main(args=None):
    rclpy.init(args=args)

    test_type = os.environ.get('TEST_TYPE')

    if test_type is None:
        logging.warning('TEST_TYPE not set. Defaulting to "small"')
        test_type = 'small'

    final_subscriber = FinalSubscriber(test_type)

    rclpy.spin(final_subscriber)

    final_subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()