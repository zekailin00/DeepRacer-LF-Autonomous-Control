find_package(ament_cmake REQUIRED)
find_package(camera_pkg REQUIRED)
ament_target_dependencies(${LF_MAIN_TARGET} camera_pkg)
