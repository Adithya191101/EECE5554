import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscroiber_node')
        self.subscriber = self.create_subscription(String, "/ad_publisher", self.heard_callback,10)

    def heard_callback(self,msg):
        self.get_logger().info(f'I recived : "{msg.data}"')
    
def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        