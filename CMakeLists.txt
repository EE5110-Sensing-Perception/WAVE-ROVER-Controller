cmake_minimum_required(VERSION 3.14)

project(wave_rover_controller LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

file(GLOB_RECURSE SRC
    ${CMAKE_SOURCE_DIR}/src/source/*.cpp
)

include_directories(${CMAKE_SOURCE_DIR}/src/include)

add_executable(wave_rover_controller ${SRC})

target_include_directories(wave_rover_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(wave_rover_controller PUBLIC cxx_std_17)

ament_target_dependencies(wave_rover_controller
  ament_index_cpp
  rclcpp
  geometry_msgs
  builtin_interfaces
)

install(TARGETS wave_rover_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include/
    DESTINATION include/${PROJECT_NAME}/
)

ament_package()
