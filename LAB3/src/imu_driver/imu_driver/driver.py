import rclpy
from rclpy.node import Node
import serial
import os
import math
from imu_msgs.msg import IMUmsg
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header

class ImuDriver(Node):
    def __init__(self):
        super().__init__('imu_driver')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.serial_port = self.get_parameter('port').get_parameter_value().string_value
        self.publisher_ = self.create_publisher(IMUmsg, 'imu', 10)

        if not os.path.exists(self.serial_port):
            self.get_logger().error(f"IMU not found on port: {self.serial_port}. Please check the connection.")
            rclpy.shutdown()
            return

        try:
            self.ser = serial.Serial(self.serial_port, baudrate=115200, timeout=1)
            self.get_logger().info(f"Opened serial port: {self.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {self.serial_port}. Error: {e}")
            rclpy.shutdown()
            return

        self.timer = self.create_timer(1.0 / 40.0, self.timer_callback)

    def timer_callback(self):
       try:
           if not self.ser.is_open:
               self.get_logger().error("Serial port is closed. Attempting to reopen...")
               self.ser.open()

           line = self.ser.readline().decode('ascii', errors='replace').strip()
           self.get_logger().debug(f"Raw data received: {line}")

           if line.startswith("$VNYMR"):
               imu_data = self.parse_vnymr(line)
               if imu_data:
                   msg = self.create_imu_message(imu_data, line)
                   self.publisher_.publish(msg)
               else:
                   self.get_logger().warn(f"Failed to parse VNYMR data: {line}")
           else:
               self.get_logger().warn(f"Received non-VNYMR data: {line}")

       except Exception as e:
           self.get_logger().error(f"Error in reading/parsing IMU data: {e}")

    def is_float(self, value):
        try:
            float(value)
            return True
        except ValueError:
            return False

    def parse_vnymr(self, line):
       try:
           # Remove checksum
           line = line.split('*')[0]
           
           # Split the data
           data = line.split(',')
           
           # Check if we have the correct number of fields
           if len(data) != 13 or not data[0].startswith("$VNYMR"):
               self.get_logger().warn(f"Invalid VNYMR data format: {line}")
               return None

           # Helper function to safely convert to float
           def safe_float(value, default=0.0):
               try:
                   return float(value.strip())
               except ValueError:
                   self.get_logger().warn(f"Failed to convert {value} to float")
                   return default

           # Parse the data
           return {
               'yaw': safe_float(data[1]),
               'pitch': safe_float(data[2]),
               'roll': safe_float(data[3]),
               'mag_x': safe_float(data[4]),
               'mag_y': safe_float(data[5]),
               'mag_z': safe_float(data[6]),
               'accel_x': safe_float(data[7]),
               'accel_y': safe_float(data[8]),
               'accel_z': safe_float(data[9]),
               'gyro_x': safe_float(data[10]),
               'gyro_y': safe_float(data[11]),
               'gyro_z': safe_float(data[12]),
           }
       except Exception as e:
           self.get_logger().error(f"Error parsing VNYMR data: {e}")
           return None

    def create_imu_message(self, imu_data, raw_data):
        msg = IMUmsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'IMU1_Frame'
        
        msg.imu.header.stamp = msg.header.stamp
        msg.imu.header.frame_id = 'IMU1_Frame'
        msg.imu.orientation = self.euler_to_quaternion(
            math.radians(imu_data['roll']),
            math.radians(imu_data['pitch']),
            math.radians(imu_data['yaw'])
        )
        
        msg.imu.angular_velocity.x = imu_data['gyro_x']
        msg.imu.angular_velocity.y = imu_data['gyro_y']
        msg.imu.angular_velocity.z = imu_data['gyro_z']
        msg.imu.linear_acceleration.x = imu_data['accel_x']
        msg.imu.linear_acceleration.y = imu_data['accel_y']
        msg.imu.linear_acceleration.z = imu_data['accel_z']
        
        msg.mag_field.header.stamp = msg.header.stamp
        msg.mag_field.header.frame_id = 'IMU1_Frame'
        
        # Convert magnetometer data from Gauss to Tesla
        gauss_to_tesla = 1e-4  # 1 Gauss = 1e-4 Tesla
        msg.mag_field.magnetic_field.x = imu_data['mag_x'] * gauss_to_tesla
        msg.mag_field.magnetic_field.y = imu_data['mag_y'] * gauss_to_tesla
        msg.mag_field.magnetic_field.z = imu_data['mag_z'] * gauss_to_tesla
        
        msg.raw_data = raw_data
        return msg

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Imu().orientation
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

def main(args=None):
    rclpy.init(args=args)
    node = ImuDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()