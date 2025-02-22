cmake_minimum_required(VERSION 3.8)
project(transport VERSION 1.0 LANGUAGES C CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# 定义选项
option(DEBUG "开启debug模式,开启后既编译为Debug模式,也会定义__DEBUG__宏,开启代码内debug逻辑,关闭后编译为Release模式,并开启-O3优化" ON)
option(USE_ROS "使用ROS,关闭后为纯cpp模式" ON)
option(USE_LOG "使用LOG,关闭后编译不再需要log库,也不产生log输出,开启默认使用标准输入输出记录log" ON)
option(USE_LOCKFREE_QUEUE "在port的消息队列中使用无锁队列" OFF)
option(USE_ROS_LOG "设置log库使用基于ROS的log" OFF)
option(USE_SPD_LOG "设置log库使用基于spd的log" OFF)
option(USE_G_LOG "设置log库使用基于glog的log" ON)
option(BUILD_TESTING "开启测试" OFF)

# 添加编译选项
add_compile_options(
    -Wno-deprecated-enum-enum-conversion 
    -Wno-unused 
    -fconcepts-diagnostics-depth=8
)

# 定义路径宏
add_definitions(
    -DTRANSPORT_CONFIG_XML_FILE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/config.xml"
    -DTRANSPORT_LOG_DIR="${PROJECT_BINARY_DIR}/../../log"
)


# 查找公共依赖项
find_package(Boost REQUIRED COMPONENTS filesystem program_options)
include_directories(${Boost_INCLUDE_DIRS})
set(PROJECT_LIBS pthread ${Boost_LIBRARIES})

if (BUILD_TESTING)
    find_package(GTest REQUIRED)
    include_directories(${GTEST_INCLUDE_DIRS})
    set(PROJECT_LIBS ${PROJECT_LIBS} GTest::GTest)
endif()

# 项目头文件目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include)

# 编译模式控制
if (DEBUG OR (CMAKE_BUILD_TYPE STREQUAL "Debug"))
    set(CMAKE_BUILD_TYPE "Debug")
    add_compile_options(-g -O0)
    add_definitions(-D__DEBUG__)
else()
    set(CMAKE_BUILD_TYPE "Release")
    add_compile_options(-O3)
endif()

# 日志库选择
if (USE_LOG)
    if (USE_ROS_LOG AND USE_ROS)
        add_definitions(-D__USE_ROS_LOG__)
    elseif (USE_SPD_LOG)
        find_package(spdlog REQUIRED)
        add_definitions(-D__USE_SPD_LOG__)
        set(PROJECT_LIBS ${PROJECT_LIBS} spdlog::spdlog)
    elseif(USE_G_LOG)
        find_package(glog REQUIRED)
        add_definitions(-D__USE_G_LOG__)
        set(PROJECT_LIBS ${PROJECT_LIBS} glog::glog)
    else()
        message(STATUS "未选择日志库,默认使用标准输入输出记录log")
    endif()
else()
    add_definitions(-D__NOT_USE_LOG__)
    message(STATUS "设置为不使用log库,将不会产生log输出")
endif()

# 无锁队列支持
if (USE_LOCKFREE_QUEUE)
    add_definitions(-DUSE_LOCKFREE_QUEUE)
endif()

# 宏定义：添加测试和工具可执行文件
macro(add_test_executable target_name)
    add_executable(${target_name} test/${target_name}.cpp)
    target_link_libraries(${target_name} ${PROJECT_LIBS})
    list(APPEND TEST_LIST ${target_name})
endmacro()

macro(add_tool_executable target_name)
    if (USE_ROS)
        ament_auto_add_executable(${target_name} tools/${target_name}.cpp)
    else()
        add_executable(${target_name} tools/${target_name}.cpp)
    endif()
    target_link_libraries(${target_name} ${PROJECT_LIBS})
    list(APPEND TOOL_LIST ${target_name})
endmacro()

# 根据 USE_ROS 决定构建方式
if (USE_ROS)
    # 查找 ROS 相关依赖
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(fmt REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(ament_cmake_auto REQUIRED)
    find_package(backward_ros REQUIRED)
    find_package(rosidl_default_generators REQUIRED)

    add_definitions(-D__USE_ROS2__)

    # 自动查找依赖
    ament_auto_find_build_dependencies()

    
    # 消息生成
    rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/GimbalPose.msg"
    "msg/Shooter.msg"
    DEPENDENCIES sensor_msgs
    )
    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
    set(PROJECT_LIBS ${PROJECT_LIBS} ${cpp_typesupport_target} fmt::fmt)
    
    if (BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        set(ament_cmake_copyright_FOUND TRUE)
        set(ament_cmake_cpplint_FOUND TRUE)
        ament_lint_auto_find_test_dependencies()
    endif()
    
    # 添加源文件
    file(GLOB_RECURSE SRC_FILES port/*.cpp pkg/*.cpp external-interface/*.cpp protocal/*.cpp)
    if (SRC_FILES)
    set(TRANSPORT_LIB ${PROJECT_NAME})
    ament_auto_add_library(${TRANSPORT_LIB} SHARED ${SRC_FILES})
    target_link_libraries(${TRANSPORT_LIB} ${PROJECT_LIBS})
    endif()
    
    # 添加主节点
    ament_auto_add_executable(${PROJECT_NAME}_node src/Transport.cpp)
    target_link_libraries(${PROJECT_NAME}_node ${PROJECT_LIBS})

    # 添加工具和测试
    add_tool_executable(KeyboardControlROS)
    add_tool_executable(clear)
    add_tool_executable(RecvPackage)
    add_tool_executable(SendPackage)
    
    if (BUILD_TESTING)
        add_test_executable(configtest)
        add_test_executable(serialIOtest)
    endif()
    
    install(DIRECTORY launch/ DESTINATION launch)

    ament_export_dependencies(rosidl_default_runtime)
    ament_package()

else()
    # 纯 C++ 模式
    file(GLOB_RECURSE SRC_FILES port/*.cpp pkg/*.cpp external-interface/*.cpp protocal/*.cpp)

    if (SRC_FILES)
        set(TRANSPORT_LIB ${PROJECT_NAME})
        add_library(${TRANSPORT_LIB} SHARED ${SRC_FILES})
        target_link_libraries(${TRANSPORT_LIB} ${PROJECT_LIBS})
    endif()

    # 主节点
    add_executable(${PROJECT_NAME}_node src/Transport.cpp)
    target_link_libraries(${PROJECT_NAME}_node ${PROJECT_LIBS})

    # 工具和测试
    add_tool_executable(clear)
    add_tool_executable(RecvPackage)
    if (BUILD_TESTING)
        add_test_executable(configtest)
        add_test_executable(serialIOtest)
        add_test_executable(buffertest)
        add_test_executable(serialbuffertest)
    endif()
    
endif()
    
# 安装规则
install(
    DIRECTORY include
    DESTINATION include/${PROJECT_NAME}
)

install(
    TARGETS
    ${PROJECT_NAME}_node
    ${TEST_LIST}
    ${TOOL_LIST}
    DESTINATION bin
)

install(DIRECTORY config/ DESTINATION config)
