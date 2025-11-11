cmake_minimum_required(VERSION 3.14)

project(wave_rover_controller LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 COMPONENTS Core SerialPort REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core SerialPort REQUIRED)

# ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Source files
file(GLOB_RECURSE SRC
    ${CMAKE_SOURCE_DIR}/src/source/*.cpp
    ${CMAKE_SOURCE_DIR}/src/include/*.hpp
)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/src/include)

# Executable target
add_executable(wave_rover_controller ${SRC})

# Target include directories
target_include_directories(wave_rover_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Compile features
target_compile_features(wave_rover_controller PUBLIC cxx_std_17)

# Dependencies
ament_target_dependencies(wave_rover_controller rclcpp geometry_msgs)
target_link_libraries(wave_rover_controller Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::SerialPort)

# Install targets
install(TARGETS wave_rover_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Install header files
install(DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include/
    DESTINATION include/${PROJECT_NAME}/
)

ament_package()

