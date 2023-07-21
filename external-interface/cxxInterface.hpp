#ifndef __CXX_INTERFACE__
#define __CXX_INTERFACE__

#include <chrono>
#include "PackageManager.hpp"

#include "pkg/GimbalPose.hpp"
#include "pkg/Shoot.hpp"

class WMJRobotControl
{
    PackageManager::SharedPtr m_package_manager;
    bool m_coor;
public:
    WMJRobotControl(PackageManager::SharedPtr package_manager)
    {
        m_package_manager = package_manager;
    }

    GimbalPose getGimbalPose()
    {
        GimbalPose gimbal_pose;
        if(m_coor)
            gimbal_pose = m_package_manager->recv<GimbalPose>(GYRO);
        else
            gimbal_pose = m_package_manager->recv<GimbalPose>(GIMBAL);
        return gimbal_pose;
    }
    void setGimbalPose(GimbalPose pose)
    {
        if(m_coor)
            m_package_manager->send(GYRO, pose);
        else
            m_package_manager->send(GIMBAL, pose);
    }

    void switchCoor(bool coor)
    {
        m_coor = coor;
    }

    void shootSome(int num = 1)
    {
        ShootPackage shoot;
        shoot.shoot_num = num;
        shoot.shoot_mode = 0;
        shoot.shoot_rub_speed = 4000;
        shoot.shoot_boost_speed = 26;
        m_package_manager->send(SHOOT, shoot);
        std::cout << shoot.toString() << std::endl;
    }

    ShootPackage getShootPackage()
    {
        auto shoot = m_package_manager->recv<ShootPackage>(SHOOT);
        return shoot;
    }
};

#endif // __CXX_INTERFACE__