#ifndef __GIMBAL_HPP__
#define __GIMBAL_HPP__

#ifdef __USE_ROS__

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "BaseROSInterface.hpp"
#include "PackageManager.hpp"
#include "pkg/GimbalPose.hpp"
#include "pkg/Shoot.hpp"
#include "pkg/GimbalControl.hpp"
#include "pkg/GimbalGlobalLink.hpp"

#include "base_interfaces/msg/gimbal_pose.hpp"

class Gimbal : public BaseROSInterface
{
public:
    Gimbal(const rclcpp::Node::SharedPtr& node, PackageManager::SharedPtr package_manager) : BaseROSInterface(node, package_manager)
    {
        addPublisher<base_interfaces::msg::GimbalPose>("gimbal-pose", 10ms, 10, std::bind(&Gimbal::publishGimbalPose, this, 0), this);
        addPublisher<base_interfaces::msg::GimbalPose>("GimbalPose", 5ms, 10, std::bind(&Gimbal::publishGimbalGlobalPose, this, 1), this);
        
        addSubscription<base_interfaces::msg::GimbalPose>("setGimbalPoseAngle", 10, std::bind(&Gimbal::subscribeGimbalPoseAngle, this, std::placeholders::_1),this);
        // addSubscription("shooting", 10, std::bind(Gimbal::subscribeShooting, this, std::placeholders::_1), this);
    }

    void publishGimbalPose(int index)
    {
        GimbalPose gimbal_pose = m_package_manager->recv<GimbalPose>(GYRO);
        m_package_manager->send(GYRO, gimbal_pose);
        std::cout << gimbal_pose.toString() << std::endl;
        base_interfaces::msg::GimbalPose msg;
        msg.pitch = gimbal_pose.pitch;
        publisher<base_interfaces::msg::GimbalPose>(index)->publish(msg);
    }

    void subscribeGimbalPoseAngle(const base_interfaces::msg::GimbalPose::SharedPtr msg)
    {
        GimbalControl gimbal_control;
        gimbal_control.info = 0x01;
        gimbal_control.pitch_angle = msg->pitch;
        gimbal_control.yaw_angle = msg->yaw;
        m_package_manager->send(GIMBAL, gimbal_control);
    }

    void publishGimbalGlobalPose(int index)
    {
        GimbalGlobalLink gimbal_pose = m_package_manager->recv<GimbalGlobalLink>(GIMBAL_GLOBAL);
        base_interfaces::msg::GimbalPose msg;
        msg.pitch = gimbal_pose.pitch_angle;
        msg.yaw = gimbal_pose.yaw_angle;
        msg.roll = gimbal_pose.roll_angle;
        publisher<base_interfaces::msg::GimbalPose>(index)->publish(msg);
    }

    // void subscribeShooting(const )
};

#endif // __USE_ROS__

#endif // __GIMBAL_HPP__