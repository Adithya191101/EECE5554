import rclpy
from rclpy.node import Node
from publisher_subscriber_pkg.srv import CheckID

class SeviceClientNode(Node):
    def __init__(self):
        super().__init__('client_node')
        self.client = self.create_client(CheckID, 'check_id')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.send_request("002055481")
    def send_request(self, id_number):
        request = CheckID.Request()
        request.id = id_number
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.response_callback)
    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Name: {response.name}')
        except Exception as e:
            self.get_logger().error(f'Sfailed in respoonse {e}')
def main(args=None):
    rclpy.init(args=args)
    node = SeviceClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()