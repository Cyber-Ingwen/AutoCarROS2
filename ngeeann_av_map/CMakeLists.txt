cmake_minimum_required(VERSION 3.5)
project(ngeeann_av_map)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

ament_auto_find_build_dependencies ()

set(DEPENDENCIES
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "std_msgs"
  "tf2"
  "tf2_ros"
  "sensor_msgs"
)

include_directories(include
  ${rclcpp_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS} 
)

add_executable(bof src/bof.cpp)
ament_target_dependencies(bof rclcpp std_msgs nav_msgs tf2 tf2_ros geometry_msgs sensor_msgs)

ament_auto_add_executable(bof_lidar src/bof_lidar.cpp)
ament_target_dependencies(bof_lidar rclcpp std_msgs nav_msgs tf2 tf2_ros geometry_msgs sensor_msgs)

target_link_libraries(bof_lidar 
    ${dependencies}
    ${PCL_LIBRARIES} 
)

install(TARGETS
  bof
  bof_lidar
  DESTINATION lib/${PROJECT_NAME})

ament_auto_package ()