cmake_minimum_required(VERSION 3.8)
project(transport)

set(CMAKE_CXX_STANDARD 20)  # 不用cxx20会编译不了

option(DEBUG "DEBUG" OFF)
option(USE_ROS "USE_ROS" ON)
option(USE_FAKE "USE_FAKE" OFF)
option(USE_LOCKFREE_QUEUE "USE_LOCKFREE_QUEUE" OFF)

# c++20弃用了enum间直接转换，有些库会报错
add_compile_options(-Wno-deprecated-enum-enum-conversion)
add_compile_options(-Wno-unused )

add_definitions(-DTRANSPORT_CONFIG_FILE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/config.yaml")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(include )
include_directories(test)

# 开启Debug编译
if(DEBUG OR (CMAKE_BUILD_TYPE AND (CMAKE_BUILD_TYPE STREQUAL "Debug")) )
  add_compile_options(-g -O0)
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
  endif()
else()
  add_compile_options(-O3)
endif()

# 打开假收发端口
if(USE_FAKE)
  add_definitions(-D__USE_FAKE__)
endif()

# 令Port使用无锁队列
if(USE_LOCKFREE_QUEUE)
  add_definitions(-DUSE_LOCKFREE_QUEUE)
  find_package(Boost REQUIRED)
endif()

################################ ROS ##########################################
if(USE_ROS) # use ros to build
# 打开ROS宏定义，编译ros部分都代码
add_definitions(-D__USE_ROS__)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(backward_ros REQUIRED)
find_package(base_interfaces REQUIRED)
find_package(libbase REQUIRED)

ament_auto_find_build_dependencies()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


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

file(GLOB_RECURSE SRC_FILES port/*.cpp pkg/*.cpp external-interface/*.cpp)
ament_auto_add_library(${PROJECT_NAME} SHARED ${SRC_FILES})

ament_auto_add_executable(${PROJECT_NAME}_node
  src/Transport.cpp
)

ament_auto_add_executable(basePackagetest
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

############################## C++ ######################################
else()  # 用纯c++

macro(add_test target_name)
  add_executable(${target_name}
    test/${target_name}.cpp
  )
  target_link_libraries(${target_name} ${PROJECT_NAME} ${OpenCV_LIBS} pthread)
endmacro()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../libbase/include)

file(GLOB_RECURSE SRC_FILES port/*.cpp pkg/*.cpp external-interface/*.cpp)
add_library(${PROJECT_NAME} SHARED ${SRC_FILES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} pthread)

add_executable(${PROJECT_NAME}_node
  src/Transport.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${OpenCV_LIBS} pthread)

add_test(basePackagetest)

endif()