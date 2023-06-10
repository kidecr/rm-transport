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

    BasePackage()=default;
    ~BasePackage()=default;

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
        auto buffer = m_buffer_queue.front();
        return buffer;
    }

    /**
     * @brief 向CanPort发送数据
     *
     */
    std::function<void(Buffer, int)> sendBuffer;

};



template <typename T>
class PackageInterFace : public BasePackage
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
        buffer = encode(package);
    }

    friend void operator<<(T &package, Buffer &buffer)
    {
        package = decode(buffer);
    }

    virtual std::string toString()
    {
        std::string str = typeid(*this).name();
        return str;
    }
};

#define ADD_SUBSCRIBE(message_type, subscriber_name) \
    rclcpp::SubScribetion<message_type>::SharedPtr subscriber_name; \
    void subscriber_name##Callback(message_type::SharedPtr msg)     \
    {                                                               \
        Buffer buffer = encode(msg);                                \
        sendBuffer(buffer, m_can_id);                               \
    }

#define ADD_PUBLISHER(message_type, publisher_name) \
    rclcpp::Publisher<message_type>::SharedPtr publisher_name;  \
    rclcpp::TimerBase publisher_name##Timer;                    \
    void publisher_name##TimerCallback()                        \
    {                                                           \
        message_type msg;                                       \
        msg = decode(m_buffer_queue);                           \
        if(publisher_name)                                      \
        {                                                       \
            publisher_name->publish(msg);                       \
        }                                                       \
    }


#define INIT_SUBSCRIBE(message_type, subscriber_name, topic_name, qos)  \
    subscriber_name = node->create_subscriber<message_type>(topic_name, qos, std::bind(&BasePackage::subscriber_name##Callback, this, std::placeholders::_1));
#define INIT_PUBLISHER(message_type, publisher_name, topic_name, qos)   \
    publisher_name = node->create_publisher<message_type>(topic_name, qos); \
    publisher_name##Timer = node->create_well_timer(10ms, std::bind(&BasePackage::publisher_name##TimerCallback, this));


#endif //__WMJ_PACKAGE_HPP__