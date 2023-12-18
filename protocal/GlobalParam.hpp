#ifndef __GLOBAL_PARAM_HPP__
#define __GLOBAL_PARAM_HPP__

#include <string>
#include <iostream>

#include "libbase/common.h"

namespace transport
{

/**
 * @brief 用于协调云台控制权限和发射权限
 * 
 */
struct ControlPermission
{
    // 云台控制权限 1是Shooter 2是Navigation 3是Scan
    wmj::GIMBAL_CONTROL_PERMISSION m_gimbalControlPermissions;
    bool m_shoot_enable;
};

} // namespace transport

#endif // __GLOBAL_PARAM_HPP__