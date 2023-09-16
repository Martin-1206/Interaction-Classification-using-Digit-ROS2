cmake_minimum_required(VERSION 3.5)
project(digit_msgs)

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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DigitMotors.msg"
  "msg/DigitJoints.msg"
  "msg/DigitState.msg"
  "msg/DigitCommand.msg"
  "msg/DigitLimits.msg"
  "msg/MobilityParameters.msg"
  "msg/RPY.msg"
  # TODO: add q and dq ??
  "action/Stand.action"
  "action/Move.action"
  "action/Pick.action"
  "action/Place.action"
  "action/Goto.action"
  "action/EndEffectorMove.action"
  "action/PointTo.action"
  "action/StartServer.action"
  "action/Bow.action"
  "action/Wave.action"
  "action/FistBump.action"
  "action/ResetEnv.action"
  "action/StartupMode.action"
  "action/OperationMode.action"
  "action/CalibrateIMU.action"
  "action/Handshake.action"
  "action/Lookfor.action"
  "action/SearchAprilTag.action"
  "action/Turn.action"
  "srv/GetLimits.srv"
  DEPENDENCIES sensor_msgs std_msgs geometry_msgs action_msgs 
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
