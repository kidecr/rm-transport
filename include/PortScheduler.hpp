#ifndef __PORT_SCHEDULER_HPP__
#define __PORT_SCHEDULER_HPP__

#include "PortManager.hpp"
#include "impls/logger.hpp"
#include "impls/Config.hpp"

#ifdef __USE_ROS2__
#include <rclcpp/rclcpp.hpp>
#endif // __USE_ROS2__

namespace transport{

class PortScheduler
{
public:
    using SharedPtr = std::shared_ptr<PortScheduler>;
private:
    PortManager::SharedPtr m_port_manager;
    std::map<std::string, std::shared_ptr<PortStatus>> m_port_status_table; // 每个端口对应的状态
    int32_t m_available_port_remained_num;
    int32_t m_max_reinit_cnt;
    std::jthread m_main_loop;
public:

    PortScheduler(config::Config::SharedPtr config, PortManager::SharedPtr port_manager)
    {
        PORT_ASSERT(port_manager != nullptr);
        m_port_manager = port_manager;
        m_available_port_remained_num = m_port_manager->getPortNum();
        m_max_reinit_cnt = config->m_reinit.m_reinit_cnt;
        LOGINFO("get avaliable port num %d", m_available_port_remained_num);

        for (auto &port_info : config->m_port_list)
        {
            std::string port_name = port_info.m_port_name;
            auto target_port = m_port_manager->m_port_table.find(port_name);
            if (target_port != m_port_manager->m_port_table.end()) // 对接口指定了分组的，给分组号，默认归到0组
            {
                if(target_port->second->activatePortScheduler()) {
                    m_port_status_table[port_name] = target_port->second->getPortStatus();
                    m_port_status_table[port_name]->group = port_info.m_group_id; // 没有唯一性检查，所以每个port的实际分组会是其所在编号最大的一个组
                }
                else{
                    LOGERROR("port scheduler activate port %s failed.", port_name.c_str());
                    throw PORT_EXCEPTION("port scheduler activate port " + port_name + " failed.");
                }
            }
        }
    }

    ~PortScheduler()
    {
        m_available_port_remained_num = 0;
    }

    /**
     * @brief 开始监控端口状态
     * 
     */
    void run()
    {
        m_main_loop = std::jthread(&PortScheduler::checkLoop, this);
        LOGINFO("check thread start");
    }

private:
    /**
     * @brief 给两个端口重新绑定回调函数, src_port => dst_port
     *
     */
    void rebindFunctionForPackage(Port::SharedPtr src_port, Port::SharedPtr dst_port)
    {
        auto src_package = src_port->m_id_map.begin();
        for (; src_package != src_port->m_id_map.end(); ++src_package)
        {
            LOGINFO("rebind package %s->%lx to %s", src_port->getPortName().c_str(), 
                        src_package->first, dst_port->getPortName().c_str());

            // port1的一个包转移到port2上
            dst_port->registerPackage(src_package->second);
            // port1重新申请一个新的包
            auto new_package = std::make_shared<BasePackage>((ID)src_package->first);
            src_port->registerPackage(new_package);
        }
    }

    /**
     * @brief 循环函数，每0.5s检查一次
     * 
     */
    void checkLoop()
    {
        while (m_available_port_remained_num && transport::ok())
        {
            checkOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 0.5秒
        }
    }

    /**
     * @brief 检查函数
     * 
     */
    void checkOnce()
    {
        LOGDEBUG("check once");
        for (auto port = m_port_status_table.begin(); port != m_port_status_table.end(); ++port)
        {
            if (port->second->status == PortStatus::Unavailable) // 该口不可用
            {
                // 如果端口可以重新唤醒，则先尝试唤醒
                if(port->second->reinit_count < m_max_reinit_cnt){
                    if(m_port_manager->m_port_table[port->first]->reinit()){
                        port->second->reinit_count += 1;
                        continue;
                    }
                }
                else{
                    port->second->status = PortStatus::Deprecated;
                    port->second->reinit_count = 0;
                }

                // 进入负载迁移流程
                LOGINFO("########## 发现不可用端口 %s  ############" , port->first.c_str());
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
                    LOGINFO("Transfer the load from %s to %s", port->second->port_name.c_str(), min_load_port->port_name.c_str());
                    // 2. 转移负载
                    auto target_port = m_port_manager->m_port_table[min_load_port->port_name];
                    auto source_port = m_port_manager->m_port_table[port->second->port_name];

                    rebindFunctionForPackage(source_port, target_port);
                    // 3. 标记源接口负载已经转移完成
                    port->second->status = PortStatus::Deprecated;
                    LOGINFO("Transfer load finished");
                }
                // 1.2 没有接口可用了
                else
                {
                    // exit
                    LOGERROR("########## 没有可用端口，退出程序 ##########");
                    // transport::shutdown();
                }
            }
        }
    }



};


} // namespace transport

#endif // __PORT_SCHEDULER_HPP__