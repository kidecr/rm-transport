cmake_minimum_required(VERSION 3.8)
project(transport VERSION 1.0 LANGUAGES C CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# 定义选项
option(DEBUG "开启debug模式,开启后既编译为Debug模式,也会定义__DEBUG__宏,开启代码内debug逻辑,关闭后编译为Release模式,并开启-O3优化" ON)
option(USE_ROS "使用ROS,关闭后为纯cpp模式" OFF)
option(USE_LOG "使用LOG,关闭后编译不再需要log库,也不产生log输出,开启默认使用标准输入输出记录log" ON)
option(USE_LOCKFREE_QUEUE "在port的消息队列中使用无锁队列" OFF)
option(USE_ROS_LOG "设置log库使用基于ROS的log" OFF)
option(USE_SPD_LOG "设置log库使用基于spd的log" OFF)
option(USE_G_LOG "设置log库使用基于glog的log" ON)
option(BUILD_TESTING "开启测试" OFF)
option(ENABLE_SERIAL_PORT "开启串口" OFF)
option(USE_PYTHON "开启python接口" OFF)
if(UNIX)
    option(ENABLE_UNIX_CAN_PORT "开启linux上的CanPort模块, 基于socketCAN" ON)
elseif(WIN32)
    option(ENABLE_WIN_BLUETOOTH "开启windows上的bluetooth模块" ON)
endif()

# 如果要使用当前环境的python,则找python包要放在vcpkg前面，否则会使用vcpkg的python
if (USE_PYTHON)
    add_definitions(-DUSE_PYTHON)
    find_package(Python COMPONENTS Interpreter Development REQUIRED)
endif()

#配置vcpkg
if(WIN32 AND NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
    include(${CMAKE_TOOLCHAIN_FILE})
endif()

if(WIN32 AND ENABLE_WIN_BLUETOOTH)
    add_definitions(-DENABLE_WIN_BLUETOOTH)
endif()

# 添加编译选项
if(MSVC)
    add_compile_options(/W4 /wd4068 /wd4996 /experimental:external /external:W0)
    # 设置 MSVC 编译器选项 /utf-8
    add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
    add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
    # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /utf-8 /GR")
    # set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /utf-8")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /MANIFESTUAC:\"level='requireAdministrator' uiAccess='false'\"")
else()
    add_compile_options(
        -Wno-deprecated-enum-enum-conversion 
        -Wno-unused 
        -fconcepts-diagnostics-depth=8
    )
endif()

# 定义路径宏
if(WIN32)
    file(TO_NATIVE_PATH "${PROJECT_BINARY_DIR}/../../log" NATIVE_LOG_DIR)
elseif(UNIX)
    file(TO_NATIVE_PATH "${PROJECT_BINARY_DIR}/log" NATIVE_LOG_DIR)
endif()
add_definitions(
    -DTRANSPORT_CONFIG_XML_FILE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/config.xml"
    -DTRANSPORT_LOG_DIR="${NATIVE_LOG_DIR}"
)

# 初始化列表变量
set(PROJECT_LIBS "")
set(TEST_LIST "")
set(TOOL_LIST "")

# 查找公共依赖项
if(UNIX)
    list(APPEND PROJECT_LIBS pthread)
endif()
find_package(Boost CONFIG REQUIRED COMPONENTS filesystem program_options)
include_directories(${Boost_INCLUDE_DIRS})
list(APPEND PROJECT_LIBS ${Boost_LIBRARIES})

if (BUILD_TESTING)
    find_package(GTest REQUIRED)
    include_directories(${GTEST_INCLUDE_DIRS})
    # list(APPEND PROJECT_LIBS )
endif()

# 如果需要构建python接口
if (USE_PYTHON)
    find_package(pybind11 CONFIG REQUIRED)
    include_directories(${pybind11_INCLUDE_DIRS})
    list(APPEND PROJECT_LIBS pybind11::module)
endif()

# 项目头文件目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include)

# 编译模式控制
# 编译模式控制
if (DEBUG OR (CMAKE_BUILD_TYPE STREQUAL "Debug"))
    set(CMAKE_BUILD_TYPE "Debug")
    if(MSVC)
        add_compile_options(/Zi /Od)
    else()
        add_compile_options(-g -O0)
    endif()
    add_definitions(-D__DEBUG__)
else()
    set(CMAKE_BUILD_TYPE "Release")
    if(MSVC)
        add_compile_options(/O2 /Ob2)
    else()
        add_compile_options(-O3)
    endif()
endif()

# 日志库选择
if (USE_LOG)
    if (USE_ROS_LOG AND USE_ROS)
        add_definitions(-D__USE_ROS_LOG__)
    elseif (USE_SPD_LOG)
        find_package(spdlog REQUIRED)
        add_definitions(-D__USE_SPD_LOG__)
        list(APPEND PROJECT_LIBS spdlog::spdlog)
    elseif(USE_G_LOG)
        find_package(glog CONFIG REQUIRED)
        # find_package(gflags CONFIG REQUIRED)
        add_definitions(-D__USE_G_LOG__)
        list(APPEND PROJECT_LIBS glog::glog )#gflags::gflags)
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
    target_link_libraries(${target_name} ${PROJECT_LIBS} GTest::GTest)
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
    list(APPEND PROJECT_LIBS ${cpp_typesupport_target} fmt::fmt)
    
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
        # add_test_executable(serialIOtest)
        add_test_executable(buffertest)
        # add_test_executable(serialbuffertest)
    endif()
    
endif()

if (USE_PYTHON)
    add_library(${PROJECT_NAME}_py MODULE src/pymodule.cpp)
    target_link_libraries(${PROJECT_NAME}_py ${PROJECT_LIBS})
    # 设置属性
    if (WIN32)
        set_target_properties(${PROJECT_NAME}_py PROPERTIES
            SUFFIX ".pyd"  # windows上后缀名为.pyd
        )
    else()
        set_target_properties(${PROJECT_NAME}_py PROPERTIES
            PREFIX ""  # 移除默认的lib前缀
            SUFFIX ".so"  # unix上后缀名为.os
        )
    endif()
    list(APPEND TOOL_LIST ${PROJECT_NAME}_py)
endif()

# 动态库符号导出（Windows需要）
if(WIN32)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
    add_definitions(-DTRANSPORT_EXPORTS)
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
install(FILES $<TARGET_FILE:Boost::filesystem>
              $<TARGET_FILE:Boost::program_options> 
              $<TARGET_FILE:glog::glog>
            #   $<TARGET_FILE:gflags::gflags>
              DESTINATION lib)

install(DIRECTORY config/ DESTINATION config)
