#!/usr/bin/env python3

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
        self.node = rclpy.create_node('talker_py')
        # declare parameter with default value
        self.node.declare_parameter('node_frequency', 1.0)
        self.freq = self.node.get_parameter(
            'node_frequency').get_parameter_value().double_value
        self.node.get_logger().info(f'Node frequency set to: {self.freq} Hz')
        self.publisher = self.node.create_publisher(String, 'chatter', 10)
        self.node.get_logger().info('Talker node initialized...')

    def start_talker(self):
        '''
        publish an incremental counter to string topic
        '''
        msg = String()
        i = 0
        rate = self.node.create_rate(self.freq)
        while rclpy.ok():
            msg.data = 'Hello World: %d' % i
            i += 1
            self.node.get_logger().info('Publishing: "%s"' % msg.data)
            self.publisher.publish(msg)
            rclpy.spin_once(self.node)
            rate.sleep() # only works if you spin once
        # Destroy the node explicitly (optional)
        self.node.destroy_node()

def main():
    rclpy.init()
    talker_node = TalkerNode()
    talker_node.start_talker()
    rclpy.shutdown()
