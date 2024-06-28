#ifndef __CXX_INTERFACE__
#define __CXX_INTERFACE__

#include <chrono>

#ifdef __USE_LIBBASE__
#include "libbase/common.h"
#endif // __USE_LIBBASE__

#include "PackageManager.hpp"
#include "protocal/Protocal.hpp"
#include "impls/PackageID.hpp"

#include "pkg/Gimbal.hpp"
#include "pkg/Shoot.hpp"
#include "pkg/TimeTest.hpp"

namespace transport{

class WMJRobotControl
{
    PackageManager::SharedPtr m_package_manager;
    bool m_coor;
public:
    WMJRobotControl(PackageManager::SharedPtr package_manager)
    {
        m_package_manager = package_manager;
    }

#ifdef __USE_LIBBASE__
    wmj::GimbalPose getGimbalPose()
    {
        wmj::GimbalPose gimbal_pose;
        GimbalPackage gimbal_package;
        if(m_coor)
            gimbal_package = m_package_manager->recv<GimbalPackage>(GYRO);
        else
            gimbal_package = m_package_manager->recv<GimbalPackage>(GIMBAL);
        gimbal_pose = gimbal_package.GetGimbalAngle();
        return gimbal_pose;
    }
    void setGimbalPose(wmj::GimbalPose pose)
    {
        GimbalPackage gimbal_package;
        gimbal_package.SetGimbalAngle(pose.pitch, pose.yaw);
        if(m_coor)
            m_package_manager->send(GYRO, gimbal_package);
        else
            m_package_manager->send(GIMBAL, gimbal_package);
    }
#endif // __USE_LIBBASE__

    void setGimbalSpeed(double x_speed, double y_speed)
    {
        GimbalPackage gimbal_package;
        gimbal_package.SetGimbalSpeed(x_speed, y_speed);
        m_package_manager->send(GIMBAL, gimbal_package);
    }

    void switchCoor(bool coor)
    {
        m_coor = coor;
    }

    int GetBulletNumber()
    {
        return m_package_manager->recv<ShootPackage>(SHOOT).getBulletNum();
    }

    void ShootSome(int num = 1)
    {
        ShootPackage shoot;
        shoot.shootSome(num);
        m_package_manager->send(SHOOT, shoot);
        // std::cout << shoot.toString() << std::endl;
    }

    void KeepShoot()
    {
        ShootPackage shoot;
        shoot.keepShoot();
        m_package_manager->send(SHOOT, shoot);
    }

    void StopShoot()
    {
        ShootPackage shoot;
        shoot.stopShoot();
        m_package_manager->send(SHOOT, shoot);
    }

    ShootPackage getShootPackage()
    {
        auto shoot = m_package_manager->recv<ShootPackage>(SHOOT);
        return shoot;
    }

    void setTime()
    {
        TimeTest t;
        static int index = 0;
        t.index = index;
        m_package_manager->send(TIME, t);
        index++;
    }

    TimeTest getTime()
    {
        return m_package_manager->recv<TimeTest>(TIME);
    }
};

} // namespace transport

#endif // __CXX_INTERFACE__