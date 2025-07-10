cmake_minimum_required(VERSION 3.8)
project(lidar_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MOSQUITTO REQUIRED libmosquitto)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(lidar_odometry_node src/lidar_odometry_node.cpp src/lidar_odometry.cpp)
ament_target_dependencies(lidar_odometry_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  laser_geometry
  pcl_conversions
  tf2
  tf2_eigen
)

target_include_directories(lidar_odometry_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(lidar_odometry_node
  ${PCL_LIBRARIES}
)

add_executable(wall_marker_node src/wall_marker_node.cpp)
ament_target_dependencies(wall_marker_node
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
)
target_link_libraries(wall_marker_node ${MOSQUITTO_LIBRARIES})
target_include_directories(wall_marker_node PRIVATE ${MOSQUITTO_INCLUDE_DIRS})
target_compile_options(wall_marker_node PRIVATE ${MOSQUITTO_CFLAGS_OTHER})

add_executable(qr_publisher src/qr_publisher.cpp)
ament_target_dependencies(qr_publisher
  rclcpp
  std_msgs
)

install(TARGETS lidar_odometry_node wall_marker_node qr_publisher
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
