#ifndef __PORT_CONTROLLER__
#define __PORT_CONTROLLER__

#include <iostream>
#include <algorithm>
#include <map>

#include <CanPort.hpp>

class PortController
{
public:
    
    std::map<std::string, CanPort::SharedPtr> m_port_map;
    std::map<std::string, std::shared_ptr<PortStatus>> m_port_status_map;    
    // std::vector<std::shared_ptr<PortStatus>> m_port_status_table;
    int m_remain_available_port_num;
    std::thread m_main_loop;

    PortController()
    {
        m_remain_available_port_num = 0;
    }

    int registerPort(CanPort::SharedPtr port)
    {
        std::string port_name = port->getPortName();
        // m_port_map[port_name] =
        if(m_port_map.find(port_name) != m_port_map.end())
        {
            std::cout << "Port name conflict !!!" << std::endl;
            return -1;
        }
        m_port_map[port_name] = port;

        auto port_status = std::make_shared<PortStatus>();
        port_status->port_name = port_name;
        port_status->status = port->isAvailable();
        port_status->workload = 0;
        m_port_status_map[port_name] = port_status;

        port->uploadAvailableStatus = std::bind(&PortController::uploadAvailableStatus, this, std::placeholders::_1, port_name);
        port->uploadWorkload = std::bind(&PortController::uploadWorkload, this, std::placeholders::_1, port_name);

        ++m_remain_available_port_num;
    }

    void uploadAvailableStatus(bool status, std::string port_name)
    {
        m_port_status_map[port_name]->status = (int)status;
    }

    void uploadWorkload(int workload, std::string port_name)
    {
        m_port_status_map[port_name]->workload = workload;
    }

    void checkLoop()
    {
        while (m_remain_available_port_num)
        {
            checkOnce();
            usleep(5e5);    // 半秒1次
        }
    }

    void checkOnce()
    {
        for(auto port = m_port_status_map.begin(); port != m_port_status_map.end(); ++port)
        {
            if(port->second->status == 0)   // 该口不可用
            {
                --m_remain_available_port_num;
                // 1. 遍历查找负担最轻的可用端口
                std::shared_ptr<PortStatus> min_load_port;
                for(auto cur_port : m_port_status_map){
                    if(cur_port.second->status != 1)
                        continue;
                    if(min_load_port == NULL)
                        min_load_port = cur_port.second;
                    if(cur_port.second->workload < min_load_port->workload)
                    {
                        min_load_port = cur_port.second;
                    }
                }

                // 1.1 选择剩余端口
                if(min_load_port)
                {
                    // 2. 转移负载
                    auto target_port = m_port_map[min_load_port->port_name];
                    auto source_port = m_port_map[port->second->port_name];
                    auto target_package_manager = target_port->getPackageManager();
                    auto source_package_manager = source_port->getPackageManager();

                    auto source_port_id_list = source_package_manager->getPortIDTable(source_port->getPortName()).id_list;
                    auto target_port_id_list = target_package_manager->getPortIDTable(target_port->getPortName()).id_list;

                    for(auto src_id : source_port_id_list)
                    {
                        // 在目标列表里找一遍，防止重复
                        int i = 0;
                        for(; i < target_port_id_list.size(); ++i)
                        {
                            if(src_id == target_port_id_list[i])
                                break;
                        }
                        // 目标列表里没找到，说明不会重复
                        if(i == target_port_id_list.size())
                        {
                            target_package_manager->bind(src_id, target_port->getPortName());   
                            if(!target_package_manager->find(src_id))
                            {
                                target_package_manager->add(source_package_manager->get(src_id));
                            }
                            target_port->registerPackage(target_package_manager->get(src_id));
                        }
                    }
                    // 3. 标记源接口负载已经转移完成
                    port->second->status = -1;
                }
                // 1.2 没有接口可用了
                else
                {
                    // exit
                    exit(-1);
                }
            }
        }
    }

    void run()
    {
        m_main_loop = std::thread(&PortController::checkLoop, this); 
    }
}


#endif // __PORT_CONTROLLER__