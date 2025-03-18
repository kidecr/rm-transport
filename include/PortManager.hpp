#ifndef __PORT_MANAGER__
#define __PORT_MANAGER__

#include <iostream>
#include <map>
#include <unordered_map>

#include "impls/Port.hpp"
#include "port/CanPort.hpp"
#include "port/SerialPort.hpp"
#include "port/WinBLEPort.hpp"

#include "utils/mask.hpp"
#include "utils/Utility.hpp"
#include "impls/logger.hpp"
#include "impls/Config.hpp"

namespace transport{

class PortManager
{
public:
    using SharedPtr = std::shared_ptr<PortManager>;

public:
    std::map<std::string, Port::SharedPtr> m_port_table;                    // 端口集合 
    std::map<std::string, std::vector<ID>> m_port_id_table;             // 每个端口对应的id列表

    PortManager(config::Config::SharedPtr config, PackageManager::SharedPtr package_manager)
    {
        PORT_ASSERT(package_manager != nullptr);
        // 1. 创建Port
        for (auto &port_info : config->m_port_list)
        {
#ifdef ENABLE_UNIX_CAN_PORT
            if(port_info.m_port_type == PORT_TYPE::CAN)
            {
                std::shared_ptr<Port> port = std::make_shared<CanPort>(port_info.m_port_name, port_info.m_group_id, 
                                                                        port_info.m_port_id, config->m_user.passwd);
                if (port)
                {
                    m_port_table[port_info.m_port_name] = port;
                    // 2.设置每个port对应的id
                    for (auto package_info : port_info.m_package_list)
                    {
                        m_port_id_table[port_info.m_port_name].push_back(package_info.m_id);
                    }
                }
                else
                {
                    LOGWARN("create port %s failed!", port_info.m_port_name.c_str());
                }
            }
            else 
#endif // ENABLE_UNIX_CAN_PORT
#ifdef ENABLE_SERIAL_PORT
            if(port_info.m_port_type == PORT_TYPE::SERIAL)
            {
                std::shared_ptr<Port> port = std::make_shared<SerialPort>(port_info.m_port_name, port_info.m_baud, 
                                                                        port_info.m_group_id, port_info.m_port_id, 
                                                                        config->m_user.passwd);
                if (port)
                {
                    m_port_table[port_info.m_port_name] = port;
                    // 2.设置每个port对应的id
                    for (auto package_info : port_info.m_package_list)
                    {
                        m_port_id_table[port_info.m_port_name].push_back(package_info.m_id);
                    }
                }
                else
                {
                    LOGWARN("create port %s failed!", port_info.m_port_name.c_str());
                }
            }
            else
#endif // ENABLE_SERIAL_PORT
#ifdef ENABLE_WIN_BLUETOOTH
            if(port_info.m_port_type == PORT_TYPE::BLUETOOTH)
            {
                std::shared_ptr<Port> port = std::make_shared<BluetoothPort>(port_info.m_port_name, 
                                                                            port_info.m_ble_service_uuid,
                                                                            port_info.m_ble_tx_characteristic_uuid,
                                                                            port_info.m_ble_rx_characteristic_uuid,
                                                                            port_info.m_group_id, 
                                                                            port_info.m_port_id, 
                                                                            config->m_user.passwd);
                if (port)
                {
                    m_port_table[port_info.m_port_name] = port;
                    // 2.设置每个port对应的id
                    for (auto package_info : port_info.m_package_list)
                    {
                        m_port_id_table[port_info.m_port_name].push_back(package_info.m_id);
                    }
                }
                else
                {
                    LOGWARN("create port %s failed!", port_info.m_port_name.c_str());
                }
            }
#endif // ENABLE_WIN_BLUETOOTH
            {
                LOGWARN("port name type illegal");
            }
        }
        // 3. 给每个端口注册包
        bindFunctionForPackage(package_manager);
    }

    /**
     * @brief 给package_list里的包绑定回调函数
     *
     * @param package_manager
     */
    void bindFunctionForPackage(PackageManager::SharedPtr package_manager)
    {
        PORT_ASSERT(package_manager != nullptr);

        for (auto &id_list : m_port_id_table)
        {
            std::string port_name = id_list.first;
            auto port = m_port_table.find(port_name);
            if (port != m_port_table.end())
            {
                for (auto id : id_list.second)
                {
                    auto package_ptr = package_manager->get(id);
                    if (package_ptr)
                    { // 存在对应的包
                        port->second->registerPackage(package_ptr);
                    }
                    else
                    {
                        LOGWARN("%s didnot find package id: %lx ", PRETTY_FUNCTION, id);
                    }
                }
            }
            else
            {
                LOGWARN("%s didnot find port %s", PRETTY_FUNCTION, port_name.c_str());
            }
        }
    }

    /**
     * @brief 根据端口名查找其id列表
     *
     * @param port_name 端口名
     * @return std::vector<ID> 找到正常返回列表，找不到返回空表
     */
    std::vector<ID> getIDList(std::string port_name)
    {
        auto id_list = m_port_id_table.find(port_name);
        if (id_list != m_port_id_table.end())
        {
            return id_list->second;
        }
        return std::vector<ID>();
    }

    /**
     * @brief 获取端口数
     * 
     * @return size_t 
     */
    size_t getPortNum()
    {
        return m_port_table.size();
    }

    /**
     * 获取每个port的名字与其id列表
    */
    std::string toString()
    {
        std::stringstream ss;
        ss << "{";
        for (auto port : m_port_id_table)
        {
            ss << "{port_name:" << port.first << ", id_list:{" << std::hex;
            for (auto id : port.second){
                ss << " 0x" << std::setw(8) << std::setfill('0') << id;
            }
            ss << "},";
        }
        ss << "}";
        return ss.str();
    }

};

} // namespace transport

#endif // __PORT_MANAGER__
