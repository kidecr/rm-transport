#ifndef __PORT_MANAGER__
#define __PORT_MANAGER__

#include <iostream>
#include <map>
#include <unordered_map>

#include <opencv2/opencv.hpp>

#include "port/CanPort.hpp"
#ifdef __USE_SERIAL_PORT__
#include "port/SerialPort.hpp"
#endif // __USE_SERIAL_PORT__

#include "utils/Utility.hpp"
#include "impls/logger.hpp"

namespace transport{

class PortManager
{
public:
    using SharedPtr = std::shared_ptr<PortManager>;

public:
    std::map<std::string, Port::SharedPtr> m_port_table;                    // 端口集合 
    std::map<std::string, std::vector<ID>> m_port_id_table;             // 每个端口对应的id列表

    PortManager(std::string config_path, PackageManager::SharedPtr package_manager)
    {
        PORT_ASSERT(package_manager != nullptr);

        cv::FileStorage fs(config_path, cv::FileStorage::READ);
        if (fs.isOpened())
        {
            // 1. 创建Port
            for (auto port_name_node : fs["port_list"])
            {
                if(port_name_node.isString())   // 内容是字符串
                {
                    std::string port_name = port_name_node;
                    if (isCanPortName(port_name)) // can
                    {
                        std::shared_ptr<Port> port = std::make_shared<CanPort>(port_name);
                        if (port)
                        {
                            m_port_table[port_name] = port;
                        }
                        else
                        {
                            LOGWARN("create port %s failed!", port_name.c_str());
                        }
                    }
                }
#ifdef __USE_SERIAL_PORT__
                else if(port_name_node.isSeq()) // 内容是列表
                {
                    std::string port_name;
                    int baud_read = 0;
                    for(auto it : port_name_node){
                        if(it.isString())
                            port_name = (std::string)it;
                        if(it.isInt())
                            baud_read = (int)it;
                    }
                    if(isSerialPortName(port_name) && baud_read != 0)
                    {
                        std::shared_ptr<Port> port = std::make_shared<SerialPort>(port_name, baud_read);
                        if (port)
                        {
                            m_port_table[port_name] = port;
                        }
                        else
                        {
                            LOGWARN("create port %s failed!", port_name.c_str());
                        }
                    }
                }
#endif // __USE_SERIAL_PORT__
                else
                {
                    LOGWARN("port name type illegal");
                }
            }
            // 2.设置每个port对应的id
            for (auto port : fs["id_list"])
            {
                std::string port_name = port["port"];
                // m_port_id_table
                if(isCanPortName(port_name))
                {
                    for (auto package_id : port["id"])
                    {
                        CAN_ID can_id = (CAN_ID)get_package_id((int)package_id);
                        ID id = mask(can_id);
                        m_port_id_table[port_name].push_back(id);
                    }
                }
                if(isSerialPortName(port_name))
                {
                    for (auto package_id : port["id"])
                    {
                        SERIAL_ID serial_id = (SERIAL_ID)get_package_id((int)package_id);
                        ID id = mask(serial_id);
                        m_port_id_table[port_name].push_back(id);
                    }
                }
            }
        }
        else
        {
            LOGERROR("Port Manager cannot open config file %s", config_path.c_str());
            throw PORT_EXCEPTION("Port Manager cannot open config file " + config_path);
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
                        LOGWARN("%s didnot find package id: %x ", __PRETTY_FUNCTION__, (int)id);
                    }
                }
            }
            else
            {
                LOGWARN("%s didnot find port %s", __PRETTY_FUNCTION__, port_name.c_str());
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

};

} // namespace transport

#endif // __PORT_MANAGER__
