#ifndef __PORT__OvO__
#define __PORT__OvO__

#include <string>
#include <unordered_map>
#include <memory>

#include <Package.hpp>
#include <PackageManager.hpp>

#define ERROR_PLACE std::string(__FILE__) + " : " + std::to_string(__LINE__) + " in function " + std::string(__FUNCTION__)

class PortException : public std::exception
{
public:
    PortException(std::string message) { this->message = message; };
    ~PortException(){};
    std::string message;
    const char* what()
    {
        if (message.empty())
            std::cout << "empty" << std::endl;
        return message.c_str();
    }
};

class Port
{
public:
    using SharedPtr = std::shared_ptr<Port>;
public:
    std::string m_port_name;
    std::unordered_map<int, std::shared_ptr<BasePackage>> m_id_map; // 包id到类成员的映射
    PackageManager::SharedPtr m_package_manager; // 包管理器
    PortStatus::SharedPtr m_port_status;    // 端口状态

    bool m_port_controller_available;   // 管理器可用
    bool m_port_is_available;           //接口可用

    BufferWithIDQueue m_write_buffer;
    std::mutex m_write_buffer_mutex;

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
        m_port_controller_available = PortStatus::Unavailable;
        m_port_is_available = false;
        m_port_name = port_name;
    }
    /**
     * @brief 注册can包, 需要自行实现
     *
     * @param package
     * @return int
     */
    virtual int registerPackage(std::shared_ptr<BasePackage> package)
    {
        if (package == nullptr)
        {
            throw PortException(ERROR_PLACE + ": package ptr is null");
            return -1;
        }
        if (package->m_can_id == 0)
        {
            throw PortException(ERROR_PLACE + ": can id is null");
            return -2;
        }

        package->sendBufferFunc = std::bind(&Port::recvBuffer, this, std::placeholders::_1, std::placeholders::_2);
        m_id_map[package->m_can_id] = package;
        return 0;
    }
    /**
     * @brief 注册包管理器，同时注册can包
     * 
     * @param package_manager 包管理器
     * @return int 
     */
    int registerPackageManager(PackageManager::SharedPtr package_manager)
    {
        if (package_manager != nullptr)
        {
            m_package_manager = package_manager;
            // 依次注册包
            auto id_table = m_package_manager->getPortIDTable(m_port_name);
            for (auto id : id_table.id_list)
            {
                auto package_ptr = m_package_manager->get(id);
                registerPackage(package_ptr);
            }
            return 0;
        }
        return -1;
    }

    /**
     * @brief 接受上层传递过来的buffer并放到缓冲区，需要自行实现
     *
     */
    void recvBuffer(Buffer buffer, int id)
    {
        BufferWithID buffer_with_id;
        buffer_with_id.first = buffer;
        buffer_with_id.second = id;

        m_write_buffer_mutex.lock();
        m_write_buffer.push(buffer_with_id);
        m_write_buffer_mutex.unlock();
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
    bool activatePortController()
    {
        if(!m_port_status)
            m_port_status = std::make_shared<PortStatus>();
        if(m_port_status)
        {
            m_port_status->port_name = m_port_name;
            m_port_controller_available = true;
            if(m_port_is_available)
                m_port_status->status = PortStatus::Available;
            else
                m_port_status->status = PortStatus::Unavailable;
        }
        return m_port_controller_available;
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
        return m_port_is_available;
    }
};

#endif // __PORT__OvO__