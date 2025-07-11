project(controller_lib)
cmake_minimum_required(VERSION 3.22.1)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(serial_driver REQUIRED)

include_directories(include)

add_library(controller_lib 
    src/controller_lib.cpp
    include/controller_lib/controller_lib.h
)

# Enable position-independent code (helped resolve -fPIC error when building)
set_target_properties(controller_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(controller_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(controller_lib serial_driver)

ament_export_include_directories(include)
ament_export_libraries(controller_lib)
ament_export_dependencies(serial_driver)

ament_package()
