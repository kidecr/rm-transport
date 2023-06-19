#ifndef __GIMBAL_HPP__
#define __GIMBAL_HPP__

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "BaseROSInterface.hpp"
#include "PackageManager.hpp"
#include "pkg/GimbalPose.hpp"
#include "pkg/Shoot.hpp"

#include "base_interfaces/msg/gimbal_pose.hpp"

class Gimbal : public BaseROSInterface
{
public:
    Gimbal(const rclcpp::Node::SharedPtr& node, PackageManager::SharedPtr package_manager) : BaseROSInterface(node, package_manager)
    {
        addPublisher<base_interfaces::msg::GimbalPose>("gimbal-pose", 10ms, 10, std::bind(&Gimbal::publishGimbalPose, this, 0), this);
        // addSubscription("shooting", 10, std::bind(Gimbal::subscribeShooting, this, std::placeholders::_1), this);
    }

    void publishGimbalPose(int index)
    {
        GimbalPose gimbal_pose = m_package_manager->recv<GimbalPose>(GIMBAL);
        m_package_manager->send(GIMBAL, gimbal_pose);
        std::cout << gimbal_pose.toString() << std::endl;
        base_interfaces::msg::GimbalPose msg;
        msg.pitch = gimbal_pose.pitch;
        publisher<base_interfaces::msg::GimbalPose>(index)->publish(msg);
    }

    // void subscribeShooting(const )
};

#endif // __GIMBAL_HPP__