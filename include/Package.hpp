#ifndef __WMJ_PACKAGE_HPP__
#define __WMJ_PACKAGE_HPP__

#include <iostream>
#include <WMJProtocol.h>
#include <queue>
#include <mutex>
#include <thread>
#include <functional>
#include <cmath>

#ifndef PI
#define PI 3.14159265358979323846
#endif // PI

/**
 * @brief 比较时间大小
 */
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

class BasePackage
{
public:
    using SharedPtr = std::shared_ptr<BasePackage>;

private:
    std::queue<BufferWithTime> m_buffer_queue; //收包的buffer队列
    timeval last_tv;
    std::mutex m_buffer_mutex;

protected:
    size_t m_max_queue_length;

public:
    CAN_ID m_can_id; // can_id

    BasePackage() = default;
    ~BasePackage() = default;

    BasePackage(CAN_ID can_id)
    {
        m_can_id = can_id;
        m_max_queue_length = 1;
    }

    BasePackage(CAN_ID can_id, size_t max_queue_length)
    {
        m_can_id = can_id;
        m_max_queue_length = max_queue_length;
    }
    /**
     * @brief 接收CanPort发来的数据
     *
     * @param buffer
     */
    void recvBuffer(BufferWithTime buffer)
    {
        timeval new_buffer_tv = buffer.second;
        if (new_buffer_tv > last_tv)
        {
            last_tv = new_buffer_tv;

            this->m_buffer_mutex.lock();
            m_buffer_queue.push(buffer);

            while (m_buffer_queue.size() > m_max_queue_length)
                m_buffer_queue.pop();
            m_buffer_mutex.unlock();
        }
    }

    /**
     * @brief 从队列取出一个buffer
     *
     * @return BufferWithTime
     */
    BufferWithTime readBuffer()
    {
        m_buffer_mutex.lock();
        auto buffer = m_buffer_queue.front();
        m_buffer_mutex.unlock();
        return buffer;
    }

    /**
     * @brief 向CanPort发送数据
     * 
     * @param buffer 数据包
     * @param id 包id
     */
    void sendBuffer(Buffer &buffer, CAN_ID id)
    {
        sendBufferFunc(buffer, id);
    }

    /**
     * @brief 向CanPort发送数据
     *
     */
    std::function<void(Buffer, int)> sendBufferFunc;
};

template <typename T>
class PackageInterFace //: public BasePackage
{
public:
    using SharedPtr = std::shared_ptr<PackageInterFace<T>>;

public:
    virtual T decode(Buffer buffer)
    {
        (void)buffer;
        T target;
        return target;
    }
    virtual Buffer encode(T target)
    {
        (void)target;
        Buffer buffer;
        return buffer;
    }

    friend void operator>>(T &package, Buffer &buffer)
    {
        buffer = package.encode(package);
    }

    friend void operator>>(Buffer &buffer, T &package)
    {
        package = package.decode(buffer);
    }

    friend void operator<<(T &package, Buffer &buffer)
    {
        package = package.decode(buffer);
    }

    friend void operator<<(Buffer &buffer, T &package)
    {
        buffer = package.encode(package);
    }

    virtual std::string toString()
    {
        std::string str = typeid(*this).name();
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
    // 1.检查是否溢出
    constexpr const size_t input_type_bits = sizeof(InputType) * 8;
    if constexpr (bits > input_type_bits)
    {
        return 0.0;
    }
    // 2.转换
    constexpr const size_t range = getNumRange<bits>();
    double target = (input & range) / range * (2 * PI);
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
    double target = angle_0_2PI<bits>(input);
    if(target > PI)
        target -= 2 * PI;
    return target;
}

/**
 * @brief 角度归一化到[min : max]区间
 * 
 * @param angle 
 * @return double 
 */
template<double min, double max>
double normalizeAngle(double angle) {
    constexpr const double range = max -min;
    while (angle >= max) {
        angle -= range;
    }
    while (angle < min) {
        angle += range;
    }
    return angle;
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
    // 防止溢出
    if (output >= (1 << bits)) {   
        output = (1 << bits) - 1;
    }
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
    unsigned long output = (unsigned long) std::round(((angle / PI) - 1) * (1 << (bits - 1)));
    // 防止溢出
    if (output >= (1 << (bits - 1))) {
        output = (1 << (bits - 1)) - 1;
    }
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
        double angle = normalizeAngle<-1 * max, max>(input);
        // 映射到[-2^(n-1), 2^(n-1)-1]
        output = (unsigned long) std::round(((angle / max) - 1) * (1 << (bits - 1)));
    }
    else {
        // 归化到[0, max]
        double angle = normalizeAngle<0.0, max>(input);
        // 映射到[0, 2^n-1]
        output = (unsigned long) std::round(angle * ((1 << bits) / (max)));
    }
    
    // 防止溢出
    if (output >= (1 << bits)) {   
        output = (1 << bits) - 1;
    }
    if(interval) {
        if(output >= (1 << (bits - 1))) {
            output -= (1 << bits);
        }
    }
    constexpr size_t range = getNumRange<bits>();
    return output & range;
}

/**
 * @brief 定义结构体，用于将结构体转成buffer
 * 
 */
#define TRANSFORM_FUNC(Type) \
friend int operator<<(Buffer &buffer, Type &msg) \
{\
    if constexpr (sizeof(Type) >= 8) return -1;\
\
    union Type2Buffer\
    {\
        Type msg;\
        uint8_t data[sizeof(Type)];\
    };\
\
    int i, size = buffer.size();\
    Type2Buffer transform;\
    transform.msg = msg;\
    \
    if(size < 8) buffer.resize(8);\
\
    for(i = 0; i < size; ++i)\
    {\
        buffer[i] = transform.data[i];\
    }\
    return sizeof(Type);\
}\
\
friend int operator<<(Type &msg, Buffer &buffer)\
{\
    if constexpr (sizeof(Type) >= 8) return -1;\
\
    union Buffer2Type\
    {\
        Type msg;\
        uint8_t data[sizeof(Type)];\
    };\
\
    int i, size = buffer.size();\
    Buffer2Type transform;\
    for(i = 0; i < size; ++i)\
    {\
        transform.data[i] = buffer[i];\
    }\
    msg = transform.msg;\
    return sizeof(Type);\
}\

#endif //__WMJ_PACKAGE_HPP__