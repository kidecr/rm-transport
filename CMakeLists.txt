cmake_minimum_required(VERSION 3.8)
project(transport)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(base_interfaces REQUIRED)

ament_auto_find_build_dependencies()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(include src pkg external-interface)

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

aux_source_directory(${CMAKE_CURRENT_LIST_DIR}/src SRC)
ament_auto_add_library(transport SHARED ${SRC})

ament_auto_add_executable(${PROJECT_NAME}_node
  test/basePackagetest.cpp
)

# 注册 导出头文件
install(
  DIRECTORY include pkg
  DESTINATION include
)

# 注册 导出可执行文件
install(
    TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
