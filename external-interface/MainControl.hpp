#ifndef __MAIN_CONTROL_HPP__
#define __MAIN_CONTROL_HPP__

#ifdef __USE_ROS__

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"

#include "BaseROSInterface.hpp"
#include "PackageManager.hpp"

#include "pkg/Judge.hpp"
#include "pkg/MainControl.hpp"

#include "base_interfaces/msg/judge.hpp"
#include "base_interfaces/msg/robot_state.hpp"

namespace transport{

class MainControl : public BaseROSInterface
{
public:
    MainControl(const rclcpp::Node::SharedPtr& node, PackageManager::SharedPtr package_manager) : BaseROSInterface(node, package_manager)
    {
        addPublisher<base_interfaces::msg::RobotState>("RobotState", 10ms, 10, std::bind(&MainControl::publishRobotState, this, 0), this);
        addPublisher<base_interfaces::msg::Judge>("RuneState", 10ms, 10, std::bind(&MainControl::publishRuneState, this, 1), this);
        addPublisher<base_interfaces::msg::Judge>("ShootSpeed", 10ms, 10, std::bind(&MainControl::publishShootSpeed, this, 2), this);

    }

    void publishRobotState(int index)
    {
        base_interfaces::msg::RobotState msg;
        auto main_control_package = m_package_manager->recv<MainControlPackage>(MAIN_CONTROL);
        msg.robot_state = main_control_package.GetRobotStatus();
        msg.camera_exposure = main_control_package.m_armor_camera_exposure;
        msg.aim_armor = main_control_package.m_aim_armor;
        msg.enable_shoot = main_control_package.m_enable_shoot;
        msg.auto_shoot = main_control_package.m_auto_shoot;

        // 没按自瞄，状态设为STATE_NONE
        if (!main_control_package.m_aim_armor)
        {
            msg.robot_state = STATE_NONE;
        }

        publisher<base_interfaces::msg::RobotState>(index)->publish(msg);
    }

    void publishRuneState(int index)
    {
        auto judge_package = m_package_manager->recv<JudgePackage>(JUDGE);
        ROBO_ENERGY rune_state = judge_package.getEnergyStatus();
        auto msg = base_interfaces::msg::Judge();
        msg.energy_state = rune_state;
        publisher<base_interfaces::msg::Judge>(index)->publish(msg);
    }

    void publishShootSpeed(int index)
    {
        auto judge_package = m_package_manager->recv<JudgePackage>(JUDGE);
        float shoot_speed = judge_package.getShootSpeedValue();
        float max_shoot_speed = judge_package.getMaxShootSpeed();
        auto msg = base_interfaces::msg::Judge();
        publisher<base_interfaces::msg::Judge>(index)->publish(msg);
    }
};

} // namespace transport

#endif // __USE_ROS__
#endif // __MAIN_CONTROL_HPP__