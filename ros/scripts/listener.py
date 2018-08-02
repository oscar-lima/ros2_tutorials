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
        super().__init__('listener')
        # setup subscriber
        self.subscription = self.create_subscription(String, 'chatter', self.listenerCallBack)


    def listenerCallBack(self, msg):
        # print received msg to console
        self.get_logger().info('I heard: "%s"' % msg.data)
        

    def start_listener(self):
        '''
        Subscribe to chatter topic and print in console its content
        '''
        # wait for ctrl + c, listen to callbacks at the same time
        rclpy.spin(self)
        # Destroy the node explicitly (optional)
        node.destroy_node()


def main():
    rclpy.init()
    listener_node = ListenerNode()
    listener_node.start_listener()
    rclpy.shutdown()


if __name__ == '__main__':
    main()        
