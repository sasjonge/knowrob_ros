cmake_minimum_required(VERSION 3.5)
project(knowrob_ros
        DESCRIPTION "A ROS wrapper for Knowrob 2.0.0"
        VERSION 1.0.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

# Message and service files
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/KeyValuePair.msg"
    "msg/GraphQueryMessage.msg"
    "msg/GraphAnswerMessage.msg"
    "msg/EventToken.msg"
    "action/AskAll.action"
    "action/AskOne.action"
    "action/AskIncremental.action"
    "action/AskIncrementalNextSolution.action"
    "action/Tell.action"
    "srv/AskIncrementalFinish.srv"
)

# Include directories
include_directories(include)

# Executables and libraries
add_executable(${PROJECT_NAME}_node src/knowrob_ros_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs spdlog)

# Install targets
install(TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME})

# Ament package
ament_package()
