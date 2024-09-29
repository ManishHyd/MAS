cmake_minimum_required(VERSION 3.8)
project(ci_agent)  # Replace PACKAGE_NAME with ci_agent, bi_agent, or visitor_agent

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # Only if you have C++ nodes, remove if you use only Python nodes
find_package(std_msgs REQUIRED)  # Adjust according to the message types used

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# if you have a C++ library
# add_library(${PROJECT_NAME} SHARED
#   src/library.cpp
# )

# if you have a C++ executable node
# add_executable(${PROJECT_NAME}_node src/node.cpp)
# ament_target_dependencies(${PROJECT_NAME}_node
#   "rclcpp"
#   "std_msgs"
# )

# if you have a Python executable node
install(
  PROGRAMS
  ci_agent/ci_agent.py  # Replace with the path to your main Python script
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
