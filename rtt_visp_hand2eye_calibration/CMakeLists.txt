cmake_minimum_required(VERSION 2.8.3)
project(rtt_visp_hand2eye_calibration)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm)

# Generate typekits for ros .msg files
ros_generate_rtt_typekit(visp_hand2eye_calibration)
# Generate the plugin which makes the services in my_msgs available
ros_generate_rtt_service_proxies(visp_hand2eye_calibration)

# Call orocos_generate_package() after the above to export the proper targets
orocos_generate_package(
  DEPENDS visp_hand2eye_calibration
  DEPENDS_TARGETS rtt_roscomm rtt_std_msgs
 rtt_geometry_msgs
)
