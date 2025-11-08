cmake_minimum_required(VERSION 3.8)
project(teleop_diffrobot)

# Khuyến nghị bật cảnh báo khi build
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(teleop_diffrobot src/teleop_diffrobot.cpp)
ament_target_dependencies(teleop_diffrobot rclcpp geometry_msgs)

install(TARGETS
  teleop_diffrobot
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

