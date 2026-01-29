import rclpy
from rclpy.node import Node
from gps_interfaces.srv import ConvertToUTM
import utm

class UTMService(Node):

    def __init__(self):
        super().__init__('utm_service')
        self.srv = self.create_service(ConvertToUTM, 'convert_to_utm', self.convert_callback)

    def convert_callback(self, request, response):
        utm_coords = utm.from_latlon(request.latitude, request.longitude)
        response.utm_easting = utm_coords[0]
        response.utm_northing = utm_coords[1]
        response.zone = utm_coords[2]
        response.letter = utm_coords[3]
        return response

def main(args=None):
    rclpy.init(args=args)
    utm_service = UTMService()

    try:
        rclpy.spin(utm_service)
    except KeyboardInterrupt:
        pass

    utm_service.destroy_node()

if __name__ == '__main__':
    main()