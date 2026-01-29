import rclpy
from rclpy.node import Node
import serial
from gps_interfaces.msg import GPSmsg
from gps_interfaces.srv import ConvertToUTM
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

class GPSDriver(Node):

    def __init__(self):
        super().__init__('gps_driver')
        
        # Declare parameters with default values
        self.declare_parameter('port', '/dev/ttyUSB0')  # Default value will be overwritten if passed via launch
        self.declare_parameter('baudrate', 4800)

        # Fetch the port parameter value
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        try:
            self.serial_connection = serial.Serial(port, baudrate, timeout=1.0)
            self.get_logger().info(f"Connected to GPS on port: {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return

        # Publisher for GPS data
        self.publisher_ = self.create_publisher(GPSmsg, '/gps', 10)
        
        # Client for UTM conversion
        self.utm_client = self.create_client(ConvertToUTM, 'convert_to_utm')

        # Timer to read from GPS puck every second
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            line = self.serial_connection.readline().decode('utf-8').strip()
        except serial.SerialException as e:
            self.get_logger().error(f"Serial port error: {e}")
            return

        if not line.startswith('$GPGGA'):
            self.get_logger().debug(f"Ignoring unsupported message: {line}")
            return

        self.get_logger().info(f"Received GPGGA data: {line}")
        
        data = line.split(',')
        
        if len(data) > 9 and data[2] and data[4] and data[9]:
            try:
                latitude = self.convert_to_decimal_degrees(data[2], data[3])
                longitude = self.convert_to_decimal_degrees(data[4], data[5])
                altitude = float(data[9])

                timestamp = self.convert_gpgga_time_to_ros_time(data[1])

                self.call_utm_service(latitude, longitude, altitude, timestamp)
            except ValueError as e:
                self.get_logger().error(f"Data conversion error: {e}")
        else:
            self.get_logger().error("Incomplete GPGGA data received.")


    def convert_to_decimal_degrees(self, coordinate_str, direction):
        if not coordinate_str:
            raise ValueError("Empty coordinate string")

        degrees = float(coordinate_str[:2] if direction in ['N', 'S'] else coordinate_str[:3])
        minutes = float(coordinate_str[2:] if direction in ['N', 'S'] else coordinate_str[3:])
        decimal_degrees = degrees + (minutes / 60.0)

        if direction in ['S', 'W']:
            decimal_degrees *= -1

        return decimal_degrees

    def convert_gpgga_time_to_ros_time(self, gpgga_time_str):
        hours = int(gpgga_time_str[0:2])
        minutes = int(gpgga_time_str[2:4])
        seconds = float(gpgga_time_str[4:])
        
        total_seconds = hours * 3600 + minutes * 60 + seconds
        
        ros_time = Time()
        ros_time.sec = int(total_seconds)
        ros_time.nanosec = int((total_seconds - int(total_seconds)) * 1e9)
        
        return ros_time

    def call_utm_service(self, latitude, longitude, altitude, timestamp):
        if not self.utm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('UTM conversion service not available')
            return

        request = ConvertToUTM.Request()
        request.latitude = latitude
        request.longitude = longitude

        future = self.utm_client.call_async(request)
        future.add_done_callback(lambda future: self.utm_callback(future, latitude, longitude, altitude, timestamp))

    def utm_callback(self, future, latitude, longitude, altitude, timestamp):
        try:
            response = future.result()
            gps_msg = GPSmsg()

            gps_msg.header = Header()
            gps_msg.header.stamp = timestamp
            gps_msg.header.frame_id = 'GPS1_Frame'

            gps_msg.latitude = latitude
            gps_msg.longitude = longitude
            gps_msg.altitude = altitude
            gps_msg.utm_easting = response.utm_easting
            gps_msg.utm_northing = response.utm_northing
            gps_msg.zone = response.zone
            gps_msg.letter = response.letter

            # Publish the message
            self.publisher_.publish(gps_msg)
            self.get_logger().info(f"Published GPS data: {gps_msg}")
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    gps_driver = GPSDriver()

    try:
        rclpy.spin(gps_driver)
    except KeyboardInterrupt:
        pass

    gps_driver.destroy_node()

if __name__ == '__main__':
    main()
