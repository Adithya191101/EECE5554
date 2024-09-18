import rclpy
from rclpy.node import Node
from publisher_subscriber_pkg.srv import CheckID
import logging

class ServiceServerNode(Node):
    def __init__(self):
        super().__init__("service_node")
        self.srv = self.create_service(CheckID, 'check_id', self.handle_service)
        logging.basicConfig(filename="id_log.csv", level=logging.INFO, format='%(message)s')

    def handle_service(self, request, response):
        if request.id == "002055481":
            response.name = "Adithya Rajendran"
            log_msg = f"{request.id},{response.name}"
        else:
            response.name = "unknown person"
            log_msg = f"{request.id},{response.name}"
        self.get_logger().info(f"NUID: {request.id}, Name: {response.name}")
        logging.info(log_msg) 
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ServiceServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
