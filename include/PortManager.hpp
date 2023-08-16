#ifndef __PORT_MANAGER__
#define __PORT_MANAGER__

#include <iostream>
#include <map>
#include <unordered_map>

#include <opencv2/opencv.hpp>

#include "port/CanPort.hpp"
#include "Utility.hpp"


class PortManager
{
public:
    using SharedPtr = std::shared_ptr<PortManager>;

public:
    std::map<std::string, Port::SharedPtr> m_port_table;                    // 端口集合 
    std::map<std::string, std::vector<CAN_ID>> m_port_id_table;             // 每个端口对应的id列表

    PortManager(std::string config_path, PackageManager::SharedPtr package_manager)
    {
        PORT_ASSERT(package_manager != nullptr);

        cv::FileStorage fs(config_path, cv::FileStorage::READ);
        if (fs.isOpened())
        {
            // 1. 创建Port
            for (auto port_name_node : fs["port_list"])
            {
                std::string port_name = port_name_node;
                if (port_name.find("can") != std::string::npos)
                {
                    std::shared_ptr<Port> port = std::make_shared<CanPort>(port_name);
                    if (port)
                    {
                        m_port_table[port_name] = port;
                    }
                    else
                    {
                        std::cout << "create port " << port_name << " failed!" << std::endl;
                    }
                }
                else
                {
                    std::cout << "port name illegal" << std::endl;
                }
            }
            // 2.设置每个port对应的id
            for (auto port : fs["id_list"])
            {
                std::string port_name = port["port"];
                // m_port_id_table
                for (auto id : port["id"])
                {
                    m_port_id_table[port_name].push_back((CAN_ID)(int)id);
                }
            }
        }
        else
        {
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
                        std::cout << __FUNCTION__ << "didnot find package id: " << std::hex << (int)id << std::endl;
                    }
                }
            }
            else
            {
                std::cout << __FUNCTION__ << " didnot find port " << port_name << std::endl;
            }
        }
    }

    /**
     * @brief 根据端口名查找其id列表
     *
     * @param port_name 端口名
     * @return std::vector<CAN_ID> 找到正常返回列表，找不到返回空表
     */
    std::vector<CAN_ID> getIDList(std::string port_name)
    {
        auto id_list = m_port_id_table.find(port_name);
        if (id_list != m_port_id_table.end())
        {
            return id_list->second;
        }
        return std::vector<CAN_ID>();
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

#endif // __PORT_MANAGER__
