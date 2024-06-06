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
    static ProcessExistsGuard process_exists_guard; // 用于在程序启动时将ProcessExists.exists置true
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

/**
 * @brief 计算两个字符串之间的编辑距离(edit distance)
 * 
 * @param str1 第一个字符串
 * @param str2 第二个字符串
 * @return int 距离
 */
int editDistance(const std::string& str1, const std::string& str2) {
    size_t len1 = str1.size(), len2 = str2.size();
    if (len1 < len2) std::swap(len1, len2);
    std::vector<int> prevRow(len2 + 1), currRow(len2 + 1);

    for (size_t j = 0; j <= len2; ++j) prevRow[j] = j;

    for (size_t i = 1; i <= len1; ++i) {
        currRow[0] = i;

        for (size_t j = 1; j <= len2; ++j) {
            int cost = (str1[i - 1] == str2[j - 1]) ? 0 : 1;
            currRow[j] = std::min({currRow[j - 1] + 1,
                                   prevRow[j] + 1,
                                   prevRow[j - 1] + cost});
        }

        prevRow.swap(currRow);
    }

    return prevRow[len2];
}

/**
 * @brief 根据编辑距离计算两个字符串之间的相似度，就是用编辑距离除以最长字符串的长度
 * 
 * @param str1 第一个字符串
 * @param str2 第二个字符串
 * @return double 相似度，0为完全不相似，1为相同
 */
double stringSimilarity(const std::string& str1, const std::string& str2) {
    int distance = editDistance(str1, str2);
    size_t maxLength = std::max(str1.size(), str2.size());
    return maxLength > 0 ? (maxLength - distance) / static_cast<double>(maxLength) : 1.0;
}

#endif // __UTILITY_HPP__