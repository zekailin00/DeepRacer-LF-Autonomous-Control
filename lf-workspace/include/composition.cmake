find_package(ament_cmake REQUIRED)

find_package(camera_pkg REQUIRED)
ament_target_dependencies(${LF_MAIN_TARGET} camera_pkg)

find_package(deepracer_interfaces_pkg REQUIRED)
ament_target_dependencies(${LF_MAIN_TARGET} deepracer_interfaces_pkg)

find_package(servo_pkg REQUIRED)
ament_target_dependencies(${LF_MAIN_TARGET} servo_pkg)

find_package(deepracer_joy_control REQUIRED)
ament_target_dependencies(${LF_MAIN_TARGET} deepracer_joy_control)

find_package(rf2o_laser_odometry REQUIRED)
find_package(tf2  REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
ament_target_dependencies(${LF_MAIN_TARGET} 
                        rf2o_laser_odometry 
                        tf2_geometry_msgs
                        geometry_msgs
                        nav_msgs
                        tf2_ros
                        tf2)

find_package(rplidar_ros2 REQUIRED)
find_package(std_srvs REQUIRED)
ament_target_dependencies(${LF_MAIN_TARGET} rplidar_ros2 std_srvs)


set(RPLIDAR_SDK_PATH "/home/zekailin00/Desktop/DeepRacer-LF-Autonomous-Control/ros-workspace/rplidar_ros/sdk")


include_directories(
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
)




