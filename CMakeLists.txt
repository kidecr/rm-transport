cmake_minimum_required(VERSION 3.8)
project(transport)

set(CMAKE_CXX_STANDARD 20) # 不用cxx20会编译不了

option(DEBUG "开启debug模式,开启后既编译为Debug模式,也会定义__DEBUG__宏,开启代码内debug逻辑,关闭后编译为Release模式,并开启-O3优化" ON)
option(USE_ROS "使用ROS,关闭后为纯cpp模式" ON)
option(USE_LOG "使用LOG,关闭后编译不再需要log库,也不产生log输出,开启默认使用glog" ON)
option(USE_LOCKFREE_QUEUE "在port的消息队列中使用无锁队列" OFF)
option(USE_ROS_LOG "设置log库使用基于ROS的log" OFF)
option(USE_SPD_LOG "设置log库使用基于spd的log" ON)
option(USE_LIBBASE "使用libbase库,用于使用lobbase中定义的数据结构" OFF)

# c++20弃用了enum间直接转换，有些库会报错
add_compile_options(-Wno-deprecated-enum-enum-conversion -Wno-unused)
# 添加concepts判定深度
add_compile_options(-fconcepts-diagnostics-depth=8)

add_definitions(-DTRANSPORT_CONFIG_FILE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/config.yaml")
add_definitions(-DTRANSPORT_CONFIG_XML_FILE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/config.xml")
add_definitions(-DTRANSPORT_LOG_DIR="${PROJECT_BINARY_DIR}/../../log")

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(include test protocal)

# 开启Debug编译
if(DEBUG OR (CMAKE_BUILD_TYPE AND (CMAKE_BUILD_TYPE STREQUAL "Debug")))
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

# 令Port使用无锁队列
if(USE_LOCKFREE_QUEUE)
  add_definitions(-DUSE_LOCKFREE_QUEUE)
endif()

if (USE_LOG)
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
else()
  add_definitions(-D__NOT_USE_LOG__)
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
    target_link_libraries(${target_name} ${PROJECT_LIB} ${OpenCV_LIBS} pthread ${LOG_LIB} ${Boost_LIBRARIES})
    set(TEST_LIST ${TEST_LIST} ${target_name})
  endmacro()

  set(TEST_LIST)
  set(PROJECT_LIB)
  file(GLOB_RECURSE SRC_FILES port/*.cpp pkg/*.cpp external-interface/*.cpp protocal/*.cpp)

  if(SRC_FILES)
    set(PROJECT_LIB ${PROJECT_NAME})
    ament_auto_add_library(${PROJECT_LIB} SHARED ${SRC_FILES})
    target_link_libraries(${PROJECT_LIB} ${OpenCV_LIBS} pthread ${LOG_LIB} ${Boost_LIBRARIES})
  endif()

  ament_auto_add_executable(${PROJECT_NAME}_node
    src/Transport.cpp
  )
  target_link_libraries(${PROJECT_NAME}_node ${OpenCV_LIBS} pthread ${LOG_LIB} ${Boost_LIBRARIES})

  add_test_executable(KeyboardControlROS)
  add_test_executable(clear)
  add_test_executable(configtest)
  add_test_executable(serialIOtest)
  add_test_executable(RecvPackage)
  add_test_executable(SendPackage)
  if(USE_LIBBASE)
    add_test_executable(EncodeDecodeTest)
  endif()

  # 注册 导出头文件
  install(
    DIRECTORY include protocal
    DESTINATION include/${PROJECT_NAME}
  )

  # 注册 导出可执行文件
  install(
    TARGETS
    ${PROJECT_NAME}_node
    ${TEST_LIST}
    DESTINATION bin
  )

  # 导出配置文件
  install(DIRECTORY
    config/
    DESTINATION config
  )

  install(DIRECTORY
    launch/
    DESTINATION launch
  )

  ament_package()

# ############################# C++ ######################################
else() # 用纯c++
  macro(add_test target_name)
    add_executable(${target_name}
      test/${target_name}.cpp
    )
    target_link_libraries(${target_name} ${PROJECT_LIB} ${OpenCV_LIBS} pthread ${LOG_LIB} ${Boost_LIBRARIES})
    set(TEST_LIST ${TEST_LIST} ${target_name})
  endmacro()
  
  if(USE_LIBBASE)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../libbase/include)
  endif()
  
  set(TEST_LIST)
  set(PROJECT_LIB)
  file(GLOB_RECURSE SRC_FILES port/*.cpp pkg/*.cpp external-interface/*.cpp protocal/*.cpp)

  if(SRC_FILES)
    set(PROJECT_LIB ${PROJECT_NAME})
    add_library(${PROJECT_LIB} SHARED ${SRC_FILES} ${LOG_LIB})
    target_link_libraries(${PROJECT_LIB} ${OpenCV_LIBS} pthread ${LOG_LIB} ${Boost_LIBRARIES})
  endif()

  add_executable(${PROJECT_NAME}_node
    src/Transport.cpp
  )
  target_link_libraries(${PROJECT_NAME}_node ${PROJECT_LIB} ${OpenCV_LIBS} pthread ${LOG_LIB} ${Boost_LIBRARIES})

  if(USE_LIBBASE)
    add_test(EncodeDecodeTest)
  endif()
  add_test(clear)
  add_test(configtest)
  add_test(RecvPackage)
  add_test(serialIOtest)

  # 注册 导出头文件
  install(
    DIRECTORY include protocal
    DESTINATION include/${PROJECT_NAME}
  )

  # 注册 导出可执行文件
  install(
    TARGETS
    ${PROJECT_NAME}_node
    ${TEST_LIST}
    DESTINATION bin
  )

  # 导出配置文件
  install(DIRECTORY
    config/
    DESTINATION config
  )

endif()