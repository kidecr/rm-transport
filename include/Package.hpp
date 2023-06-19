#ifndef __WMJ_PACKAGE_HPP__
#define __WMJ_PACKAGE_HPP__

#include <iostream>
#include <WMJProtocol.h>
#include <queue>
#include <mutex>
#include <thread>
#include <functional>

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

#endif //__WMJ_PACKAGE_HPP__