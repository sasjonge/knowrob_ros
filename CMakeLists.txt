cmake_minimum_required(VERSION 3.5)
project(knowrob_ros VERSION 0.1.0 LANGUAGES CXX)

# ——— C++ Standard ————————————————————
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wswitch -pthread -Wno-undef")

# ——— find dependencies —————————————————
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(spdlog REQUIRED)
# If you have a ROS 2 port of knowrob installed:
find_package(knowrob REQUIRED)

# ——— Interface definition  ———————————
# Generate messages, services, and actions
rosidl_generate_interfaces(${PROJECT_NAME}
  # messages
  "msg/KeyValuePair.msg"
  "msg/ModalFrame.msg"
  "msg/Triple.msg"
  "msg/TellMessage.msg"
  "msg/GraphQueryMessage.msg"
  "msg/GraphAnswerMessage.msg"
  # services
  "srv/AskIncrementalFinish.srv"
  "srv/ExportTriples.srv"
  # actions
  "action/AskAll.action"
  "action/AskOne.action"
  "action/AskIncremental.action"
  "action/AskIncrementalNextSolution.action"
  "action/Tell.action"
  DEPENDENCIES std_msgs action_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# ——— Include directories ————————————
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

# ——— C++ node ——————————————————————
add_executable(knowrob_ros_node src/ROSInterface.cpp)
ament_target_dependencies(knowrob_ros_node
  rclcpp
  std_msgs
  spdlog::spdlog
  knowrob
)
# Link generated type support into your node:
rosidl_target_interfaces(knowrob_ros_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(
  TARGETS knowrob_ros_node
  DESTINATION lib/${PROJECT_NAME}
)

# ——— Python scripts (installation) —————————
# Installs all executables in scripts/ into the package share directory
install(
  DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}/scripts
  FILES_MATCHING PATTERN "*.py"
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
              GROUP_READ GROUP_EXECUTE
              WORLD_READ WORLD_EXECUTE
)

# ——— Testing (optional) ——————————————
if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # e.g. ament_add_pytest_test(…)
endif()

# ——— ament package macro —————————————
ament_package()
