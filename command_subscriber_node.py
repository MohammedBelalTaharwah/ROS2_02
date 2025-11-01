#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # لاستقبال رسالة من نوع String

class CommandSubscriberNode(Node):
    """
    عقدة تشترك (subscribe) في التوبيك /robot/command وتطبع الأوامر المستلمة.
    """
    def __init__(self):
        super().__init__('command_subscriber_node')
        # إنشاء مشترك (subscriber)
        self.subscription = self.create_subscription(
            String,
            '/robot/command',
            self.listener_callback, # الدالة التي ستنفذ عند وصول رسالة
            10)
        
        self.get_logger().info('Command Subscriber node has been started and is listening.')

    def listener_callback(self, msg):
        command = msg.data.lower()
        self.get_logger().info('Received command: "%s"' % command)
        
        # معالجة الأوامر
        if command == 'start':
            self.get_logger().info('Processing: START command executed.')
        elif command == 'stop':
            self.get_logger().info('Processing: STOP command executed.')
        elif command == 'reset':
            self.get_logger().info('Processing: RESET command executed.')
        else:
            self.get_logger().warn('Unknown command received: "%s"' % command)

def main(args=None):
    rclpy.init(args=args)
    
    command_subscriber = CommandSubscriberNode()
    
    try:
        rclpy.spin(command_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        command_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
