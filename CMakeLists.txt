cmake_minimum_required(VERSION 3.8)
project(mecha_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#-- Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nucleo_agent REQUIRED)

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MechaState.msg"
  "msg/PointAndMechaState.msg"
  "msg/PointAndMechaStateArray.msg"
  "msg/DaizaCmdType.msg"
  "msg/HinaCmdType.msg"
  "action/DaizaCmd.action"
  "action/HinaCmd.action"
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

#-- for sequence_controller
# find_package(mecha_control REQUIRED)
add_executable(cmd_seq
  src/cmd_seq.cpp
)
target_include_directories(cmd_seq PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(cmd_seq "${cpp_typesupport_target}")
ament_target_dependencies(cmd_seq
  rclcpp
  rclcpp_action
  std_srvs
  rosidl_default_generators
  nucleo_agent
)
install(TARGETS cmd_seq
  DESTINATION lib/${PROJECT_NAME}
)
#-- end sequence_controller

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

#-- for Python
# ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  mecha_control/mecha_control_node.py
  mecha_control/dummy_node.py
  mecha_control/debug_controller.py
  mecha_control/debug_sequence_controller.py
  mecha_control/path_command_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)
#--- end Python

ament_package()
