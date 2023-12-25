#ifndef __GLOBAL_PARAM_HPP__
#define __GLOBAL_PARAM_HPP__

#include <string>
#include <iostream>

#ifdef __USE_LIBBASE__
#include "libbase/common.h"
#endif // __USE_LIBBASE__

namespace transport
{

#ifdef __USE_LIBBASE__
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
#endif // __USE_LIBBASE__
} // namespace transport

// #endif // __GLOBAL_PARAM_HPP__