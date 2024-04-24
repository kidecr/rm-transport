#ifndef __WORK_LOAD_HPP__
#define __WORK_LOAD_HPP__

#include <sys/time.h>
#include <iostream>
#include <memory>
#include <string>

#include "utils/Defines.hpp"

namespace transport{

/**
 * @brief 接口负载
 * 
 */
class Workload
{
private:
    time_t m_last_sec;
    int m_last_sec_count;
    int m_cur_count;
    bool m_update;

public:
    /**
     * @brief 更新一次接口负载，本质上就是负载加一
     * 
     */
    void update()
    {
        timeval tv;
        gettimeofday(&tv, NULL);
        if (m_last_sec == tv.tv_sec)
            ++m_cur_count;
        else
        {
            m_update = true;
            m_last_sec = tv.tv_sec;
            m_last_sec_count = m_cur_count;
            m_cur_count = 0;
        }
    }

    /**
     * @brief 获取上一秒负载都值
     * 
     * @return int 负载值
     */
    int getWorkload()
    {
        m_update = false;
        return m_last_sec_count;
    }

    bool canUpload()
    {
        return m_update;
    }

    int operator=(int workload)
    {
        m_last_sec_count = workload;
        return workload;
    }

    int operator+(Workload &workload)
    {
        return m_last_sec_count + workload.m_last_sec_count;
    }

    operator int()
    {
        m_update = false;
        return m_last_sec_count;
    }

    friend std::ostream &operator<<(std::ostream &ostream, Workload &workload)
    {
        ostream << workload.m_last_sec_count;
        return ostream;
    }
};

/**
 * @brief 接口负载
 * 
 */
class PortWorkloads
{
public:
    Workload read;
    Workload write;

public:
    operator int()
    {
        int read_workload = read.getWorkload();
        int write_workload = write.getWorkload();
        return read_workload + write_workload;
    }

    int operator=(int workload)
    {
        read = workload;
        write = 0;
        return workload;
    }

    bool operator<(PortWorkloads &workload)
    {
        int A_workload = read + write;
        int B_workload = workload.read + workload.write;
        return A_workload < B_workload;
    }

    bool operator>(PortWorkloads &workload)
    {
        int A_workload = read + write;
        int B_workload = workload.read + workload.write;
        return A_workload > B_workload;
    }
};

// 接口信息表
class PortStatus
{
public:
    using SharedPtr = std::shared_ptr<PortStatus>;
    // 这么写不会占用空间
    IENUM Available = 1;    // 可用
    IENUM Unavailable = 0;  // 不可用
    IENUM Deprecated = -1;  // 弃用
public:
    std::string port_name;  // 接口名
    int status;             // 可用状态 1:可用, 0:不可用, -1:该口已经迁移完成,彻底弃用
    int group;              // 接口所在组别
    int reinit_count;       // 端口重启次数
    PortWorkloads workload; // 接口工作负载

public:
    PortStatus(): status{Unavailable}, group{0} {}
};

/**
 * @brief 判断输入是否为一个can设备名
 * 
 * @param port_name 设备名
 * @return true 是can设备名
 * @return false 不是can设备名
 */
bool isCanPortName(std::string& port_name)
{
    return port_name.find("can") != std::string::npos;
}

/**
 * @brief 判断输入是否为一个serial设备名
 * 
 * @param port_name 设备名
 * @return true 是serial设备名
 * @return false 不是serial设备名
 */
bool isSerialPortName(std::string& port_name)
{
    return port_name.find("tty") != std::string::npos || port_name.find("pts") != std::string::npos;
}

}   // namespace transport

#endif // __WORK_LOAD_HPP__