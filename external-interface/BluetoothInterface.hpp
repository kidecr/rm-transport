#ifndef __BLUETOOTH_INTERFACE__
#define __BLUETOOTH_INTERFACE__

#define ENABLE_WIN_BLUETOOTH
#ifdef ENABLE_WIN_BLUETOOTH

#include <chrono>
#include <thread>

#include "PackageManager.hpp"
#include "PortManager.hpp"
#include "PortScheduler.hpp"
#include "protocal/Protocal.hpp"
#include "impls/PackageID.hpp"

#include "pkg/wtIMU.hpp"
#include <pybind11/pybind11.h>

class BluetoothInterface
{
private:
    transport::config::Config::SharedPtr m_config;
    transport::PackageManager::SharedPtr m_package_manager;
    transport::PortManager::SharedPtr m_port_manager;
    transport::PortScheduler::SharedPtr m_port_scheduler;
    std::jthread m_thread;
    
public:
    BluetoothInterface(std::string config_path)
    {
        LOGINIT("BluetoothInterface");
        m_config = std::make_shared<transport::config::Config>(config_path);
        m_package_manager = std::make_shared<transport::PackageManager>(m_config);
        m_port_manager = std::make_shared<transport::PortManager>(m_config, m_package_manager);
        m_port_scheduler = std::make_shared<transport::PortScheduler>(m_config, m_port_manager);
        m_thread = std::jthread([this]() { 
            std::this_thread::sleep_for(std::chrono::seconds(5));
            m_port_scheduler->run(); 
        });
    }
#ifdef USE_PYTHON
    pybind11::dict recvIMU()
    {
        auto imu_package = m_package_manager->recv<transport::WTIMU>(WT_BLT_RX);
        pybind11::dict imu_package_dict;
        imu_package_dict["ax"] = imu_package.m_ax;
        imu_package_dict["ay"] = imu_package.m_ay;
        imu_package_dict["az"] = imu_package.m_az;
        imu_package_dict["wx"] = imu_package.m_wx;
        imu_package_dict["wy"] = imu_package.m_wy;
        imu_package_dict["wz"] = imu_package.m_wz;
        imu_package_dict["roll"] = imu_package.m_roll;
        imu_package_dict["pitch"] = imu_package.m_pitch;
        imu_package_dict["yaw"] = imu_package.m_yaw;
        return imu_package_dict;
    }
#endif // USE_PYTHON

    std::vector<std::string> getAvailablePortName()
    {
        return m_port_manager->getAvailablePortName();
    }

    transport::WTIMU recvWTIMU() {
        LOGINFO("WT_BLT_RX %X", WT_BLT_RX.id);
        return m_package_manager->recv<transport::WTIMU>(WT_BLT_RX);
    }
};

#endif // ENABLE_WIN_BLUETOOTH
#endif // __BLUETOOTH_INTERFACE__