#!/usr/bin/env python3

from time import sleep

import rclpy
from std_msgs.msg import String

# NOTE: no rclpy.node.Node inheritance
class TalkerNode:
    def __init__(self):
        '''
        publisher example in ROS2
        constructor
        '''
        # init node
        self.node = rclpy.create_node('talker')
        self.publisher = self.node.create_publisher(String, 'chatter', 10)
        self.node.get_logger().info('Talker node initialized...')


    def start_talker(self):
        '''
        publish an incremental counter to string topic
        '''
        msg = String()
        i = 0
        while rclpy.ok():
            msg.data = 'Hello World: %d' % i
            i += 1
            self.node.get_logger().info('Publishing: "%s"' % msg.data)
            self.publisher.publish(msg)
            sleep(0.5)  # seconds
        # Destroy the node explicitly (optional)
        node.destroy_node()


def main():
    rclpy.init()
    talker_node = TalkerNode()
    talker_node.start_talker()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
