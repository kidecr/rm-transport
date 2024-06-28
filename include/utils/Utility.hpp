#ifndef __UTILITY_HPP__
#define __UTILITY_HPP__

#include <iostream>
#include <time.h>

#include "utils/Defines.hpp"
#include "utils/Buffer.hpp"
#include "utils/PortRelated.hpp"
#include "utils/SystemRelated.hpp"

#include "impls/exception.hpp"
#include "protocal/Protocal.hpp"
#include "protocal/GlobalParam.hpp"

#ifdef __USE_ROS2__
#include <rclcpp/rclcpp.hpp>
#endif // __USE_ROS2__ 

/**************************************类**********************************************/

namespace transport{

/**
 * @brief
 *
 */
class ProcessExistsGuard
{
public:
    ProcessExistsGuard()
    {
        GET_PARAM(ProcessExists)->exists = true;
#ifdef __USE_ROS2__
        if(!rclcpp::ok()){
            auto init_options = rclcpp::InitOptions();
            rclcpp::init(0, nullptr, init_options);
        }
#endif // __USE_ROS2__
    }

    ~ProcessExistsGuard()
    {
        GET_PARAM(ProcessExists)->exists = false;
    }
};

/**
 * @brief 判断当前是否处于正常状态
 */
bool ok()
{
    static ProcessExistsGuard process_exists_guard; // 用于在程序启动时将ProcessExists.exists置true
#ifndef __USE_ROS2__
    return GET_PARAM(ProcessExists)->exists;
#else 
    return GET_PARAM(ProcessExists)->exists && rclcpp::ok();
#endif // __USE_ROS2__
}

void shutdown()
{
    GET_PARAM(ProcessExists)->exists = false;
#ifdef __USE_ROS2__
    rclcpp::shutdown();
#endif // __USE_ROS2__
}


} // namespace transport

/**************************************函数********************************************/


//@brief 用cout输出buffer， 16进制
std::ostream& operator <<(std::ostream &stream, transport::Buffer &buffer)
{
    stream << std::hex;
    for(size_t i = 0; i < buffer.size(); ++i)
        stream << (uint32_t)buffer[i] << " ";
    stream << std::dec;
    return stream;
}


//@brief 比较时间大小
static bool operator>(const timeval& t1, const timeval& t2)
{
    if (t1.tv_sec > t2.tv_sec)
        return true;
    else if (t1.tv_sec < t2.tv_sec)
        return false;
    else
        return t1.tv_usec > t2.tv_usec;
}

//@brief 比较时间大小
static bool operator>(const timeval& t1, const double& sec)
{
    // 首先检查秒数是否大于
    time_t isec = static_cast<time_t>(sec);
    if (t1.tv_sec > isec) {
        return true;
    }
    // 如果秒数相等，则检查微秒数
    else if (t1.tv_sec == isec) {
        // 将 sec 的小数部分与 tv_usec 比较
        double fsec = sec - isec;
        time_t usec = static_cast<time_t>(fsec * 1e6);
        return t1.tv_usec > usec;
    }
    // 如果秒数小于，则肯定不满足条件
    else {
        return false;
    }
}

//@brief 比较时间大小
static bool operator<(const timeval& t1, const timeval& t2)  
{  
    if (t1.tv_sec < t2.tv_sec)  
        return true;  
    else if (t1.tv_sec > t2.tv_sec)  
        return false;  
    else  
        return t1.tv_usec < t2.tv_usec;  
}

//@brief 比较时间大小
static bool operator>=(const timeval& t1, const timeval& t2)
{
    if (t1.tv_sec > t2.tv_sec)
        return true;
    else if (t1.tv_sec == t2.tv_sec)
        return t1.tv_usec >= t2.tv_usec;
    else
        return false;
}

//@brief 比较时间大小
static bool operator<=(const timeval& t1, const timeval& t2)  
{  
    if (t1.tv_sec < t2.tv_sec)  
        return true;  
    else if (t1.tv_sec == t2.tv_sec)  
        return t1.tv_usec <= t2.tv_usec;  
    else  
        return false;  
}

//@brief 比较时间是否相等
static bool operator==(const timeval& t1, const timeval& t2)
{
    if (t1.tv_sec == t2.tv_sec && t1.tv_usec == t2.tv_usec)
        return true;
    else
        return false;
    return false;
}

//@brief 计算时间差
static timeval operator-(const timeval& t1, const timeval& t2)
{
    timeval t3;  
    t3.tv_sec = t1.tv_sec - t2.tv_sec;  
    t3.tv_usec = t1.tv_usec - t2.tv_usec;  
  
    // 如果微秒数结果为负，则向秒数借位  
    if (t3.tv_usec < 0) {  
        t3.tv_sec -= 1;  
        t3.tv_usec += 1000000;  
    }  
  
    // 如果t1小于t2，则结果应为负时间差  
    // 这里我们不需要再次处理微秒数的进位，因为上面的借位处理已经足够  
    if (t1 < t2) {  
        t3.tv_sec = -t3.tv_sec;  
        t3.tv_usec = -t3.tv_usec;  
    }  
  
    return t3;  
}

//@brief cout输出timeval
static std::ostream& operator <<(std::ostream &stream, timeval &tv)
{
    stream << "sec:" << tv.tv_sec << " usec:" << tv.tv_usec;
    return stream;
}



#endif // __UTILITY_HPP__