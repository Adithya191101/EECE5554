cmake_minimum_required(VERSION 3.5)
project(imu_msgs)

# Find dependencies
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/IMUmsg.msg"
  DEPENDENCIES std_msgs sensor_msgs
)

# Export message dependencies to other packages
ament_export_dependencies(rosidl_default_runtime)

# Finalize the package
ament_package()
