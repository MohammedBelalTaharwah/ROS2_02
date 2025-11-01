#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # لاستخدام رسالة من نوع String

class StatusPublisherNode(Node):
    """
    عقدة تقوم بنشر حالة الروبوت (Status) كل ثانيتين.
    """
    def __init__(self):
        super().__init__('status_publisher_node')
        # إنشاء ناشر (publisher) على التوبيك /robot/status
        self.publisher_ = self.create_publisher(String, '/robot/status', 10)
        
        # إنشاء مؤقت (timer) لتشغيل دالة timer_callback كل 2 ثانية
        timer_period = 2.0  # ثانية
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Status Publisher node has been started and is publishing.')

    def timer_callback(self):
        msg = String()
        # محتوى الرسالة
        msg.data = 'Robot Status: OK | Timestamp: %s' % self.get_clock().now().to_msg()
        
        # نشر الرسالة
        self.publisher_.publish(msg)
        
        # طباعة رسالة في الطرفية (Terminal)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    
    status_publisher = StatusPublisherNode()
    
    try:
        rclpy.spin(status_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # تدمير العقدة وإغلاق rclpy
        status_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
