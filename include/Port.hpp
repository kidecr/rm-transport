#ifndef __PORT__OvO__
#define __PORT__OvO__

#include <string>
#include <unordered_map>
#include <memory>

#include "BasePackage.hpp"
#include "PackageManager.hpp"
#include "Utility.hpp"


class Port
{
public:
    using SharedPtr = std::shared_ptr<Port>;
public:
    std::string m_port_name;
    std::unordered_map<int, std::shared_ptr<BasePackage>> m_id_map; // 包id到类成员的映射
protected:
    PackageManager::SharedPtr m_package_manager; // 包管理器
    PortStatus::SharedPtr m_port_status;    // 端口状态

    bool m_port_sheduler_available;   // 管理器可用
    bool m_port_is_available;           //接口可用

    BufferWithIDQueue m_write_buffer;   
    std::mutex m_write_buffer_mutex;
    IENUM MAX_WRITE_BUFFER_SIZE = 10;
#ifdef USE_LOCKFREE_QUEUE
    std::atomic<int> m_write_buffer_size;
#endif // USE_LOCKFREE_QUEUE

    /**
     * @brief 从写队列中读一个buffer
     * 
     * @param buffer_with_id 
     * @return true 
     * @return false 
     */
    bool popOneBuffer(BufferWithID &buffer_with_id)
    {
#ifndef USE_LOCKFREE_QUEUE
        std::lock_guard lock(m_write_buffer_mutex);
        if(m_write_buffer.empty()) return false;
        buffer_with_id = m_write_buffer.front();
        m_write_buffer.pop();
        return true;
#else 
        if(m_write_buffer.pop(buffer_with_id)){
            --m_write_buffer_size;
            return true;
        }
        return false;
#endif // USE_LOCKFREE_QUEUE
    }
    /**
     * @brief 向写队列中写一个buffer
     * 
     * @param buffer_with_id 
     * @return true 
     * @return false 
     */
    bool pushOneBuffer(BufferWithID &buffer_with_id) {
#ifndef USE_LOCKFREE_QUEUE
        std::lock_guard lock(m_write_buffer_mutex);
        m_write_buffer.push(buffer_with_id);
        while (m_write_buffer.size() > MAX_WRITE_BUFFER_SIZE) {
            m_write_buffer.pop();
        }
        return true;
#else
        bool success = m_write_buffer.push(buffer_with_id);
        if(success)
            ++m_write_buffer_size;

        while (m_write_buffer_size > MAX_WRITE_BUFFER_SIZE)
        {
            BufferWithID bwi;
            if(m_write_buffer.pop(bwi))
                --m_write_buffer_size;
        }
        
        return success;
#endif // USE_LOCKFREE_QUEUE
    }

public:
    Port()=default;
    ~Port() = default;

    /**
     * @brief 初始化端口名，默认所有状态为false
     * 
     * @param port_name 端口名
     */
    Port(std::string port_name)
    {
        m_port_sheduler_available = PortStatus::Unavailable;
        m_port_is_available = false;
        m_port_name = port_name;
#ifdef USE_LOCKFREE_QUEUE
        m_write_buffer_size = 0;
#endif // USE_LOCKFREE_QUEUE
    }
    /**
     * @brief 注册can包, 如果原来就有对应id，则更新指针并重绑函数，否则新增指针
     *
     * @param package
     * @return int
     */
    virtual int registerPackage(std::shared_ptr<BasePackage> package)
    {
        if (package == nullptr)
        {
            throw PORT_EXCEPTION("package ptr is null");
            return -1;
        }
        if (package->m_can_id == 0)
        {
            throw PORT_EXCEPTION("can id is null");
            return -2;
        }

        package->sendBufferFunc = std::bind(&Port::recvBuffer, this, std::placeholders::_1, std::placeholders::_2);
        m_id_map[package->m_can_id] = package;
        return 0;
    }

    /**
     * @brief 接受上层传递过来的buffer并放到缓冲区，需要自行实现
     *
     */
    void recvBuffer(Buffer buffer, int id)
    {
        BufferWithID buffer_with_id;
        buffer_with_id.buffer = buffer;
        buffer_with_id.id = id;

        pushOneBuffer(buffer_with_id);
    }

    /**
     * @brief 获取端口名
     * 
     * @return std::string 
     */
    std::string getPortName()
    {
        return m_port_name;
    }
    /**
     * @brief 获取端口状态指针
     * 
     * @return std::shared_ptr<PortStatus> 
     */
    std::shared_ptr<PortStatus> getPortStatus()
    {
        return m_port_status;
    }
    /**
     * @brief 代表端口控制相关功能打开
     * 
     * @return true 正常打开
     * @return false 打开失败
     */
    bool activatePortSheduler()
    {
        if(!m_port_status)
            m_port_status = std::make_shared<PortStatus>();
        if(m_port_status)
        {
            m_port_status->port_name = m_port_name;
            m_port_sheduler_available = true;
            if(m_port_is_available)
                m_port_status->status = PortStatus::Available;
            else
                m_port_status->status = PortStatus::Unavailable;
        }
        else {
            m_port_sheduler_available = false;
            m_port_status->status = PortStatus::Unavailable;
        }
        return m_port_sheduler_available;
    }
    /**
     * @brief 返回package_manager
     * 
     * @return PackageManager::SharedPtr 
     */
    PackageManager::SharedPtr getPackageManager()
    {
        return m_package_manager;
    }
    /**
     * @brief 当前接口是否可用
     * 
     * @return true 可用
     * @return false 不可用
     */
    bool portIsAvailable()
    {
        return m_port_is_available == PortStatus::Available;
    }
};

#endif // __PORT__OvO__