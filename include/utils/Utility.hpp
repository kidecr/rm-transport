#ifndef __UTILITY_HPP__
#define __UTILITY_HPP__

#include <iostream>
#include <time.h>

#include "utils/Defines.hpp"
#include "utils/Buffer.hpp"
#include "utils/PortRelated.hpp"
#include "utils/SystemRelated.hpp"
#include "utils/mask.hpp"

#include "impls/exception.hpp"
#include "protocal/Protocal.hpp"
#include "protocal/GlobalParam.hpp"

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
    }

    ~ProcessExistsGuard()
    {
        GET_PARAM(ProcessExists)->exists = false;
    }
};

/**
 * @brief
 */
bool ok()
{
    static ProcessExistsGuard process_exists_guard;
    return GET_PARAM(ProcessExists)->exists;
}

void shutdown()
{
    GET_PARAM(ProcessExists)->exists = false;
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
static bool operator>(timeval t1, timeval t2)
{
    if (t1.tv_sec > t2.tv_sec)
        return true;
    else if (t1.tv_sec == t2.tv_sec && t1.tv_usec > t2.tv_usec)
        return true;
    else
        return false;
    return false;
}

//@brief 比较时间大小
static bool operator>=(timeval t1, timeval t2)
{
    if (t1.tv_sec > t2.tv_sec)
        return true;
    else if (t1.tv_sec == t2.tv_sec && t1.tv_usec >= t2.tv_usec)
        return true;
    else
        return false;
    return false;
}

//@brief 比较时间是否相等
static bool operator==(timeval t1, timeval t2)
{
    if (t1.tv_sec == t2.tv_sec && t1.tv_usec == t2.tv_usec)
        return true;
    else
        return false;
    return false;
}

//@brief 计算时间差
static timeval operator-(timeval t1, timeval t2)
{
    if(t1 >= t2) {
        timeval t3;
        t3.tv_sec = t1.tv_sec - t2.tv_sec;
        t3.tv_usec = t1.tv_usec - t2.tv_usec;
        return t3;
    }
    else {
        timeval t3 = t2 - t1;
        t3.tv_sec = 0 - t3.tv_sec;
        t3.tv_usec = 0 - t3.tv_usec;
        return t3;
    }
}
//@brief cout输出timeval
static std::ostream& operator <<(std::ostream &stream, timeval &tv)
{
    stream << "sec:" << tv.tv_sec << " usec:" << tv.tv_usec;
    return stream;
}



#endif // __UTILITY_HPP__