cmake_minimum_required(VERSION 3.22)
project(manipulator_hw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial_driver REQUIRED)

# ADDED FOR THE CONTROLLER LIBRARY
add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../controller_lib
  ${CMAKE_BINARY_DIR}/controller_lib_build
)
# END

# ADDED THE FOLLOWING
# plugin library definition
add_library(${PROJECT_NAME} SHARED
  src/manipulator.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  # points to source include dir
  $<INSTALL_INTERFACE:include>  # points to installed headers
)

# link other libraries to plugin
# ADDED FOR THE CONTROLLER LIBRARY
target_link_libraries(${PROJECT_NAME} controller_lib)
# END

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
)

# make plugin available
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME} # export file so that the plugin can be found/used by other packages
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib # installs compiled library to lib dir
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY 
    launch 
    config
    urdf
  DESTINATION share/${PROJECT_NAME}/
)

install(
  FILES manipulator.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(hardware_interface manipulator.xml)
# END OF ADDED

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ADDED THE FOLLOWING
# export dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(hardware_interface pluginlib rclcpp)
# END OF ADDED 

ament_package()