#ifndef __PORT_CONTROLLER__
#define __PORT_CONTROLLER__

#include <iostream>
#include <algorithm>
#include <map>

#include <CanPort.hpp>
#include <WMJProtocol.h>

#include <opencv2/opencv.hpp>

class PortController
{
public:
    
    std::map<std::string, CanPort::SharedPtr> m_port_map;
    std::map<std::string, std::shared_ptr<PortStatus>> m_port_status_map;    
    std::map<std::string, int> m_port_group;
    // std::vector<std::shared_ptr<PortStatus>> m_port_status_table;
    int m_available_port_remained_num;
    std::thread m_main_loop;

    PortController()
    {
        m_available_port_remained_num = 0;
        cv::FileStorage fs("../config/PackageList.yaml", cv::FileStorage::READ);
        int i = 0;
        for(auto group : fs["port_group"])
        {
            for(auto port : group)
            {
                std::string port_name = port;
                m_port_group[port_name] = i;
            }
            i++;
        }
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

        port->activatePortController();
        m_port_map[port_name] = port;
        m_port_status_map[port_name] = port->getPortStatus();
        if(m_port_group.find(port_name) != m_port_group.end())
            m_port_status_map[port_name]->group = m_port_group[port_name];
        else
            m_port_status_map[port_name]->group = 0;

        ++m_available_port_remained_num;
        return 0;
    }

    void checkLoop()
    {
        while (m_available_port_remained_num)
        {
            checkOnce();
            usleep(5e5);    // 半秒1次
        }
    }

    void checkOnce()
    {
        std::cout << "!! check once\n";
        for(auto port = m_port_status_map.begin(); port != m_port_status_map.end(); ++port)
        {
            if(port->second->status == PortStatus::Unavailable)   // 该口不可用
            {
                std::cout << "########## 发现不可用端口 ############" << std::endl;
                std::cout << "port name is " << port->first << std::endl;
                --m_available_port_remained_num;
                // 1. 遍历查找负担最轻的可用端口
                std::shared_ptr<PortStatus> min_load_port = NULL;
                for(auto cur_port : m_port_status_map){ // 只能在同一个组内查找可替代端口
                    if(cur_port.second->status != PortStatus::Available || cur_port.second->group != port->second->group)
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
                    //! TODO : 重写逻辑
                    auto target_port = m_port_map[min_load_port->port_name];
                    auto source_port = m_port_map[port->second->port_name];
                    auto target_package_manager = target_port->getPackageManager();
                    auto source_package_manager = source_port->getPackageManager();

                    auto source_port_id_list = source_package_manager->getPortIDTable(source_port->getPortName()).id_list;
                    auto target_port_id_list = target_package_manager->getPortIDTable(target_port->getPortName()).id_list;

                    for(auto src_id : source_port_id_list)
                    {
                        // 在目标列表里找一遍，防止重复
                        size_t i = 0;
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
                    port->second->status = PortStatus::Deprecaped;
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
        m_main_loop.detach();
    }
};


#endif // __PORT_CONTROLLER__