#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    def __init__(self):
        '''
        subscriber example in ROS2
        constructor
        '''
        # set node name
        super().__init__('listener_py')
        # setup subscriber
        self.subscription = self.create_subscription(String, 'chatter', self.listenerCallBack, 10)
        self.get_logger().info('Listener node initialized...')

    def listenerCallBack(self, msg):
        # print received msg to console
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    listener_node = ListenerNode()
    rclpy.spin(listener_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    #             when the garbage collector destroys the node object)
    listener_node.destroy_node()
    rclpy.shutdown()
