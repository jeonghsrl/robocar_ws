cmake_minimum_required(VERSION 3.5)
project(cont_cmdvel)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)                    #added
find_package(geometry_msgs REQUIRED)               #added

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(exec_cmd_vel src/pub_cmd_vel.cpp)   #added
ament_target_dependencies(exec_cmd_vel rclcpp std_msgs geometry_msgs)  #added

install(TARGETS                              #added  
  exec_cmd_vel                               #executor of cmd_vel publisher 
  DESTINATION lib/${PROJECT_NAME})

ament_package()
