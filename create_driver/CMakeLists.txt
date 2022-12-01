cmake_minimum_required(VERSION 3.8)
project(create_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(libcreate REQUIRED)

find_package(rclcpp REQUIRED)
find_package(create_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

#find_package(rosidl_default_generators REQUIRED)

#roslint_cpp()
#roslint_add_test()

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  create_msgs
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  libcreate
)

target_include_directories(${PROJECT_NAME} PUBLIC
 "${libcreate_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME}
 "${libcreate_LIBRARIES}")

install(TARGETS ${PROJECT_NAME}
  #ARCHIVE DESTINATION lib
  #LIBRARY DESTINATION lib
  #RUNTIME DESTINATION lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

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

ament_export_include_directories(include)
ament_export_dependencies(
  create_msgs
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  libcreate
)
ament_package()
