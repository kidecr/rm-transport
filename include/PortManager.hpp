#ifndef __PORT_MANAGER__
#define __PORT_MANAGER__

#include "CanPort.hpp"
#include "Utility.hpp"
#include <opencv2/opencv.hpp>

#include <iostream>
#include <map>
#include <unordered_map>

class PortManager
{
public:
    using SharedPtr = std::shared_ptr<PortManager>;

public:
    std::map<std::string, Port::SharedPtr> m_port_table;                    // 端口集合
    std::map<std::string, std::shared_ptr<PortStatus>> m_port_status_table; // 每个端口对应的状态            // 每个端口对应都分组
    std::map<std::string, std::vector<CAN_ID>> m_port_id_table;             // 每个端口对应的id列表

    int m_available_port_remained_num;
    bool m_enbale_port_shedule;
    std::thread m_main_loop;
    PackageManager::SharedPtr m_package_manager;

    PortManager(std::string config_path, bool enable_port_shedule = true)
    {
        m_enbale_port_shedule = enable_port_shedule;
        m_available_port_remained_num = 0;
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
                        ++m_available_port_remained_num;
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
            // 2. 设置分组
            if (m_enbale_port_shedule)
            {
                int i = 0;
                for (auto group : fs["shedule_group"])
                {
                    for (auto port : group)
                    {
                        std::string port_name = port;
                        auto target_port = m_port_table.find(port_name);
                        if (target_port != m_port_table.end()) // 对接口指定了分组的，给分组号，默认归到0组
                        {
                            target_port->second->activatePortController();
                            m_port_status_table[port_name] = target_port->second->getPortStatus();
                            m_port_status_table[port_name]->group = i; // 没有唯一性检查，所以每个port的实际分组会是其所在编号最大的一个组
                        }
                    }
                    i++;
                }
            }
            // 3.设置每个port对应的id
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
            throw PortException("Port controller cannot open config file!");
        }
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
     * @brief 给两个端口重新绑定回调函数, src_port => dst_port
     *
     */
    void rebindFunctionForPackage(Port::SharedPtr src_port, Port::SharedPtr dst_port)
    {
        auto src_package = src_port->m_id_map.begin();
        for (; src_package != src_port->m_id_map.end(); ++src_package)
        {
            // port1的一个包转移到port2上
            dst_port->registerPackage(src_package->second);
            // port1重新申请一个新的包
            auto new_package = std::make_shared<BasePackage>((CAN_ID)src_package->first);
            src_port->registerPackage(new_package);
        }
    }

    void checkLoop()
    {
        while (m_available_port_remained_num && m_enbale_port_shedule)
        {
            checkOnce();
            usleep(5e5); // 半秒1次
        }
    }

    void checkOnce()
    {
        std::cout << "!! check once\n";
        for (auto port = m_port_status_table.begin(); port != m_port_status_table.end(); ++port)
        {
            if (port->second->status == PortStatus::Unavailable) // 该口不可用
            {
                std::cout << "########## 发现不可用端口 " << port->first << " ############" << std::endl;
                --m_available_port_remained_num;
                // 1. 遍历查找负担最轻的可用端口
                std::shared_ptr<PortStatus> min_load_port = NULL;
                for (auto cur_port : m_port_status_table)
                { // 只能在同一个组内查找可替代端口
                    if (cur_port.second->status != PortStatus::Available || cur_port.second->group != port->second->group)
                        continue;
                    if (min_load_port == NULL)
                        min_load_port = cur_port.second;
                    if (cur_port.second->workload < min_load_port->workload)
                    {
                        min_load_port = cur_port.second;
                    }
                }

                // 1.1 选择剩余端口
                if (min_load_port)
                {
                    // 2. 转移负载
                    auto target_port = m_port_table[min_load_port->port_name];
                    auto source_port = m_port_table[port->second->port_name];

                    rebindFunctionForPackage(source_port, target_port);
                    // 3. 标记源接口负载已经转移完成
                    port->second->status = PortStatus::Deprecaped;
                }
                // 1.2 没有接口可用了
                else
                {
                    // exit
                    std::cout << "########## 没有可用端口，退出程序 ##########" << std::endl;
                    exit(-1);
                }
            }
        }
    }

    void run()
    {
        if (m_enbale_port_shedule)
        {
            m_main_loop = std::thread(&PortManager::checkLoop, this);
            m_main_loop.detach();
        }
    }
};

#endif // __PORT_MANAGER__
