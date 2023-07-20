#ifndef __CXX_INTERFACE__
#define __CXX_INTERFACE__

#include <chrono>
#include "PackageManager.hpp"

#include "pkg/GimbalPose.hpp"

class WMJRobotControl
{
    PackageManager::SharedPtr m_package_manager;
public:
    WMJRobotControl(PackageManager::SharedPtr package_manager)
    {
        m_package_manager = package_manager;
    }

    GimbalPose getGimbalPose()
    {
        GimbalPose gimbal_pose = m_package_manager->recv<GimbalPose>(GYRO);
        return gimbal_pose;
    }
    void setGimbalPose(GimbalPose pose)
    {
        m_package_manager->send(GYRO, pose);
    }
};

#endif // __CXX_INTERFACE__