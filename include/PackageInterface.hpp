#ifndef __PACKAGE_INTERFACE_HPP__
#define __PACKAGE_INTERFACE_HPP__

#include <iostream>
#include <string>
#include <cxxabi.h>

#include "BasePackage.hpp"
#include "Utility.hpp"

namespace transport{

template <typename T>
class PackageInterFace
{
public:
    using SharedPtr = std::shared_ptr<PackageInterFace<T>>;

public:
    virtual void decode(T &target, Buffer &buffer)
    {
        (void)buffer;
        (void)target;
        return;
    }
    virtual void encode(T &target, Buffer &buffer)
    {
        (void)buffer;
        (void)target;
        return;
    }

    friend void operator>>(T &package, Buffer &buffer)
    {
        package.encode(package, buffer);
    }

    friend void operator>>(Buffer &buffer, T &package)
    {
        package.decode(package, buffer);
    }

    friend void operator<<(T &package, Buffer &buffer)
    {
        package.decode(package, buffer);
    }

    friend void operator<<(Buffer &buffer, T &package)
    {
        package.encode(package, buffer);
    }

    virtual std::string toString()
    {
        std::string str = abi::__cxa_demangle(typeid(*this).name(), 0, 0, 0);
        return str;
    }

};


/**
 * @brief 获得对应bit位的全1的数
 * 
 * @tparam bits 位宽
 * @return unsigned long 
 */
template<int bits>
constexpr inline unsigned long getNumRange()
{
    return (0x01UL << bits) - 1;
}

/**
 * @brief 角度归一化到[min : max]区间
 * 
 * @param angle 
 * @return double 
 */
template<double min, double max>
double normalizeAngle(double angle) {
    constexpr const double range = max - min;
    while (angle >= max) {
        angle -= range;
    }
    while (angle < min) {
        angle += range;
    }
    return angle;
}

/**
 * @brief 将输入类型按照buffer解析，将从0开始的bits位转为0-2PI范围的角度
 * 
 * @tparam bits 要读取的范围
 * @tparam InputType 输入类型，可以为uint8 uint16 uint32等
 * @param input 输入数据
 * @return double 0-2PI范围的角度
 */
template<int bits, typename InputType>
double angle_0_2PI(InputType input)
{
    // 0.检查输入的数据类型
    if constexpr (!std::is_integral<InputType>::value)
    {
        std::cout << "InputType is not integral!" << std::endl;
        return 0.0;
    }
    // 1.检查是否溢出
    constexpr const size_t input_type_bits = sizeof(InputType) * 8;
    if constexpr (bits > input_type_bits)
    {
        std::cout << "tparam bits overflow" << std::endl;
        return 0.0;
    }
    // 2.转换
    constexpr const size_t range = getNumRange<bits>();
    double target = (input & range) * (2 * PI) / range;
    return target;
}

/**
 * @brief 将输入类型按照buffer解析，将从0开始的bits位转为-PI-PI范围的角度
 * 
 * @tparam bits 要读取的范围
 * @tparam InputType 输入类型，可以为uint8 uint16 uint32等
 * @param input 输入数据
 * @return double -PI-PI范围的角度
 */
template<int bits, typename InputType>
double angle_navPI_PI(InputType input)
{
    // 0.检查输入的数据类型
    if constexpr (!std::is_integral<InputType>::value)
    {
        std::cout << "InputType is not integral!" << std::endl;
        return 0.0;
    }
    // 1.转换
    double target = angle_0_2PI<bits>(input);
    if(target > PI)
        target -= 2 * PI;
    return target;
}

/**
 * @brief 将buffer转化为
 * 
 * @tparam bits 
 * @tparam max 
 * @tparam interval true: [-max, max] false: [0, max]
 * @tparam InputType 
 * @param input 
 * @return double 
 */
template<int bits, double max, bool interval, typename InputType>
double angle_max_interval(InputType input)
{
    // 0.检查输入的数据类型
    if constexpr (!std::is_integral<InputType>::value)
    {
        std::cout << "InputType is not integral!" << std::endl;
        return 0.0;
    }
    // 1.检查是否溢出
    constexpr const size_t input_type_bits = sizeof(InputType) * 8;
    if constexpr (bits > input_type_bits)
    {
        std::cout << "tparam bits overflow" << std::endl;
        return 0.0;
    }
    // 2.转换
    constexpr const size_t range = getNumRange<bits>();
    double target = 0.0;

    if constexpr (interval)    // [-max, max]
    {
        target = (input & range) * (2.0 * max) / range;
        if(target > max)
            target -= 2.0 * max;
    }
    else //[0, max]
    {
        target = (input & range) * max / range;
    }
    return target;
}

/**
 * @brief 将角度归一化到[0,2PI],然后转成bits大小的buffer
 * 
 * @tparam bits 
 * @param input 
 * @return unsigned long 
 */
template<int bits>
unsigned long buffer_0_2PI(double input)
{
    // 超出范围
    if constexpr (bits > 63) {
        return 0UL;
    }
    // 归化到[0, 2PI]
    double angle = normalizeAngle<0.0, 2 * PI>(input);
    // 映射到[0, 2^n-1]
    unsigned long output = (unsigned long) std::round(angle * ((1 << bits) / (2 * PI)));

    constexpr size_t range = getNumRange<bits>();
    return output & range;
}

/**
 * @brief 将角度归一化到[-PI,PI],然后转成bits大小的buffer
 * 
 * @tparam bits 
 * @param input 
 * @return unsigned long 
 */
template<int bits>
unsigned long buffer_navPI_PI(double input)
{
    // 超出范围
    if constexpr (bits > 63) {
        return 0UL;
    }
    // 归化到[-PI, PI]
    double angle = normalizeAngle<-PI, PI>(input);
    // 映射到[-2^(n-1), 2^(n-1)-1]
    unsigned long output = (unsigned long) std::round(angle * (1 << (bits - 1)) / (2 * PI));

    constexpr size_t range = getNumRange<bits>();
    return output & range;
}

/**
 * @brief 如果interval为真，将输入归一化到[-max,max]，为假则归一化到[0,max]
 * 
 * @tparam bits 位宽
 * @tparam max 区间大小
 * @tparam interval 判断是否为正负区间，true: [-max,max] false: [0,max]
 * @param input 
 * @return unsigned long 
 */
template<int bits, double max, bool interval>
unsigned long buffer_max_interval(double input)
{
    // 超出范围
    if constexpr (bits > 63) {
        return 0UL;
    }
    unsigned long output = 0UL;
    if constexpr (interval) {
        // 归化到[-max, max]
        double angle = normalizeAngle<0.0 - max, max>(input);
        // 映射到[-2^(n-1), 2^(n-1)-1]
        output = (unsigned long) std::round(angle * ((1 << (bits - 1)) - 1) / (2 * max));
    }
    else {
        // 归化到[0, max]
        double angle = normalizeAngle<0.0, max>(input);
        // 映射到[0, 2^n-1]
        output = (unsigned long) std::round(angle * ((1 << bits) / (max)));
    }
    
    constexpr size_t range = getNumRange<bits>();
    return output & range;
}

/**
 * @brief 用于结构体和buffer间的相互转换
 * 
 */
#define TRANSFORM_FUNC(Type) \
\
friend int operator<<(Buffer &buffer, Type &msg) \
{\
    uint8_t *data = (uint8_t *)&msg;\
    size_t i, size = sizeof(Type);\
\
    buffer.resize(size);\
\
    if constexpr (std::is_same<Buffer, std::vector<uint8_t>>::value)\
    {\
        for(i = 0; i < size; ++i)\
        {\
            buffer[i] = data[i];\
        }\
    }\
    else\
    {\
        memmove(buffer.data, data, size);\
        buffer.length = size;\
    }\
    return sizeof(Type);\
}\
\
friend int operator<<(Type &msg, Buffer &buffer)\
{\
    uint8_t *data = (uint8_t *)&msg;\
    size_t i, size = buffer.size();\
\
    size = size <= sizeof(Type) ? size : sizeof(Type);\
\
    if constexpr (std::is_same<Buffer, std::vector<uint8_t>>::value)\
    {\
        for(i = 0; i < size; ++i)\
        {\
            data[i] = buffer[i];\
        }\
    }\
    else\
    {\
        memmove(data, buffer.data, size);\
    }\
    return sizeof(Type);\
}\
\
inline uint8_t& operator[](int index)\
{\
    if(index >= (int)sizeof(Type) || index < 0)\
        throw PORT_EXCEPTION("index out of range");\
\
    uint8_t* data = (uint8_t*)this; \
    return data[index]; \
}\


} // namespace transport
#endif // __PACKAGE_INTERFACE_HPP__