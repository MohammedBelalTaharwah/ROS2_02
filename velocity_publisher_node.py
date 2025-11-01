#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist # لاستخدام رسالة السرعة Twist

class VelocityPublisherNode(Node):
    """
    عقدة تقوم بنشر أوامر سرعة (Twist) على التوبيك /cmd_vel.
    """
    def __init__(self):
        super().__init__('velocity_publisher_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # سنقوم بالنشر مرة واحدة كمثال، أو يمكن ربطه بمؤقت
        timer_period = 1.0  # انشر كل ثانية
        self.timer = self.create_timer(timer_period, self.publish_velocity)
        
        self.get_logger().info('Velocity Publisher node has been started.')

    def publish_velocity(self):
        msg = Twist()
        
        # مثال: اجعل الروبوت يتحرك للأمام بسرعة خطية 0.5 م/ث
        msg.linear.x = 0.5
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        
        # مثال: اجعل الروبوت يدور بسرعة زاوية 0.1 راد/ث
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.1 
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing velocity: Linear=%.2f, Angular=%.2f' % (msg.linear.x, msg.angular.z))


def main(args=None):
    rclpy.init(args=args)
    
    velocity_publisher = VelocityPublisherNode()
    
    try:
        rclpy.spin(velocity_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        velocity_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
