cmake_minimum_required(VERSION 3.8)
project(transport)

set(CMAKE_CXX_STANDARD 20) # 不用cxx20会编译不了

option(DEBUG "DEBUG" OFF)
option(USE_ROS "USE_ROS" ON)
option(USE_FAKE "USE_FAKE" OFF)
option(USE_LOCKFREE_QUEUE "USE_LOCKFREE_QUEUE" OFF)
option(USE_ROS_LOG "USE_ROS_LOG" OFF)
option(USE_SPD_LOG "USE_SPD_LOG" ON)
option(USE_SERIAL_PORT "USE_SERIAL_PORT" ON)
option(USE_LIBBASE "USE_LIBBASE" OFF)

# c++20弃用了enum间直接转换，有些库会报错
add_compile_options(-Wno-deprecated-enum-enum-conversion -Wno-unused)
# 添加concepts判定深度
add_compile_options(-fconcepts-diagnostics-depth=8)

add_definitions(-DTRANSPORT_CONFIG_FILE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/config.yaml")
add_definitions(-DTRANSPORT_LOG_DIR="${PROJECT_BINARY_DIR}/../../log")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(include test protocal)

# 开启Debug编译
if(DEBUG OR(CMAKE_BUILD_TYPE AND(CMAKE_BUILD_TYPE STREQUAL "Debug")))
  add_compile_options(-g -O0)
  add_definitions(-D__DEBUG__)

  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
  endif()
else()
  add_compile_options(-O3)

  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
  endif()
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
if(USE_SERIAL_PORT)
  add_definitions(-D__USE_SERIAL_PORT__)
  find_package(Boost REQUIRED)
endif()

# 默认使用glog，当设置USE_ROS_LOG或USE_SPD_LOG时，优先级 ROS > spd
if(USE_ROS_LOG AND USE_ROS)
  add_definitions(-D__USE_ROS_LOG__)
  set(LOG_LIB)
  set(USE_SPD_LOG OFF)
else()
  if(USE_SPD_LOG)
    find_package(spdlog REQUIRED)
    add_definitions(-D__USE_SPD_LOG__)
    set(LOG_LIB spdlog) # 直接装的spdlog是head-only的，编译安装的spdlog是有静态库的
  else()
    # 默认使用glog
    find_package(glog REQUIRED)
    set(LOG_LIB glog)
  endif()
endif()

if(USE_LIBBASE)
  add_definitions(-D__USE_LIBBASE__)
endif()

# ############################### ROS ##########################################
if(USE_ROS) # use ros to build
  # 打开ROS宏定义，编译ros部分都代码
  add_definitions(-D__USE_ROS2__)

  # find dependencies
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(ament_cmake_auto REQUIRED)
  find_package(backward_ros REQUIRED)
  find_package(base_interfaces REQUIRED)
  if(USE_LIBBASE)
    find_package(libbase REQUIRED)
  endif()

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

  macro(add_test_executable target_name)
    ament_auto_add_executable(${target_name}
      test/${target_name}.cpp
    )
    target_link_libraries(${target_name} ${PROJECT_LIB} ${OpenCV_LIBS} pthread ${LOG_LIB})
    set(TEST_LIST ${TEST_LIST} ${target_name})
  endmacro()

  set(TEST_LIST)
  set(PROJECT_LIB)
  file(GLOB_RECURSE SRC_FILES port/*.cpp pkg/*.cpp external-interface/*.cpp)

  if(SRC_FILES)
    set(PROJECT_LIB ${PROJECT_NAME})
    ament_auto_add_library(${PROJECT_LIB} SHARED ${SRC_FILES})
    target_link_libraries(${PROJECT_LIB} ${OpenCV_LIBS} pthread ${LOG_LIB})
  endif()

  ament_auto_add_executable(${PROJECT_NAME}_node
    src/Transport.cpp
  )
  target_link_libraries(${PROJECT_NAME}_node ${OpenCV_LIBS} pthread ${LOG_LIB})

  add_test_executable(KeyboardControlROS)
  add_test_executable(clear)
  if(USE_LIBBASE)
    add_test_executable(EncodeDecodeTest)
  endif()

  # 注册 导出头文件
  install(
    DIRECTORY include pkg
    DESTINATION include
  )

  # 注册 导出可执行文件
  install(
    TARGETS
    ${PROJECT_NAME}_node
    ${TEST_LIST}
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY
    launch
    config
    DESTINATION share/${PROJECT_NAME}
  )

  ament_package()

# ############################# C++ ######################################
else() # 用纯c++
  macro(add_test target_name)
    add_executable(${target_name}
      test/${target_name}.cpp
    )
    target_link_libraries(${target_name} ${PROJECT_LIB} ${OpenCV_LIBS} pthread ${LOG_LIB})
  endmacro()
  
  if(USE_LIBBASE)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../libbase/include)
  endif()
  
  set(PROJECT_LIB)
  file(GLOB_RECURSE SRC_FILES port/*.cpp pkg/*.cpp external-interface/*.cpp)

  if(SRC_FILES)
    set(PROJECT_LIB ${PROJECT_NAME})
    add_library(${PROJECT_LIB} SHARED ${SRC_FILES} ${LOG_LIB})
    target_link_libraries(${PROJECT_LIB} ${OpenCV_LIBS} pthread ${LOG_LIB})
  endif()

  add_executable(${PROJECT_NAME}_node
    src/Transport.cpp
  )
  target_link_libraries(${PROJECT_NAME}_node ${PROJECT_LIB} ${OpenCV_LIBS} pthread ${LOG_LIB})

  if(USE_LIBBASE)
    add_test(EncodeDecodeTest)
  endif()
  add_test(clear)
endif()