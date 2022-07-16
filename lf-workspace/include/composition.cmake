find_package(ament_cmake REQUIRED)

find_package(camera_pkg REQUIRED)
ament_target_dependencies(${LF_MAIN_TARGET} camera_pkg)

find_package(deepracer_interfaces_pkg REQUIRED)
ament_target_dependencies(${LF_MAIN_TARGET} deepracer_interfaces_pkg)

find_package(servo_pkg REQUIRED)
ament_target_dependencies(${LF_MAIN_TARGET} servo_pkg)

find_package(deepracer_joy_control REQUIRED)
ament_target_dependencies(${LF_MAIN_TARGET} deepracer_joy_control)
