import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from datetime import datetime

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node') 
        self.publisher_ = self.create_publisher(String, '/ad_publisher', 10)
        self.timer = self.create_timer(1.0, self.publisher_msg)
 
    def publisher_msg(self):
        time = datetime.now().strftime("%H:%M:%S")
        day = datetime.now().strftime("%A")
        msg = String()
        msg.data = f"adithya is publishing data on time: {time}, day: {day}"
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
