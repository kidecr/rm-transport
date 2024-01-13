#ifndef __GIMBAL_HPP__
#define __GIMBAL_HPP__

#ifdef __USE_ROS2__

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"

#include "impls/BaseROSInterface.hpp"
#include "PackageManager.hpp"
#include "protocal/GlobalParam.hpp"

#include "pkg/Gimbal.hpp"

#include "base_interfaces/msg/gimbal_pose.hpp"
#include "base_interfaces/msg/scan_ctrl_info.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

namespace transport{

class Gimbal : public BaseROSInterface
{
private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_jointStatePublisher;
public:
    Gimbal(const rclcpp::Node::SharedPtr& node, PackageManager::SharedPtr package_manager) : BaseROSInterface(node, package_manager)
    {
        // gimbal 控制
        addPublisher<base_interfaces::msg::GimbalPose>("GetGimbalAngle", 5ms, 10, std::bind(&Gimbal::publishGimbalAngle, this, 0), this);
        addPublisher<base_interfaces::msg::GimbalPose>("GetGimbalSpeed", 5ms, 10, std::bind(&Gimbal::publishGimbalSpeed, this, 1), this);
    
        addSubscription<base_interfaces::msg::GimbalPose>("SetGimbalAngle", 10, std::bind(&Gimbal::setGimbalAngleCallback, this, std::placeholders::_1), this);
        addSubscription<base_interfaces::msg::GimbalPose>("SetGimbalSpeed", 10, std::bind(&Gimbal::setGimbalSpeedCallback, this, std::placeholders::_1), this);
        addSubscription<base_interfaces::msg::GimbalPose>("SetGimbal_YawSpeed_PitchAngle", 10, std::bind(&Gimbal::setGimbal_YawSpeed_PitchAngle_Callback, this, std::placeholders::_1), this);
        // scan 云台权限切换
        addSubscription<base_interfaces::msg::ScanCtrlInfo>("ScanCtrlInfo", 10, std::bind(&Gimbal::scanSubscriptionCallback, this, std::placeholders::_1), this);

        m_jointStatePublisher = m_node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);
    }


    //接受者的回调函数，直接把收到的信息发给了电控
    //需要使用的参数类型请看函数声明
    /**
     * @brief 使云台以最高速度转到目标角度
     *
     * @param base_interfaces::msg::GimbalPose msg. float msg.pitch俯仰角度,float msg.yaw  偏航角度
     */
    void setGimbalAngleCallback(const base_interfaces::msg::GimbalPose::SharedPtr msg)
    {
        float pitch_angle = msg->pitch;
        float yaw_angle = msg->yaw;
        GimbalPackage gimbal_package;
        gimbal_package.SetGimbalAngle(pitch_angle, yaw_angle);
        m_package_manager->send(GIMBAL, gimbal_package);
    }
    /**
     * @brief 设定云台的角速度
     *
     * @param float msg.pitch 俯仰速度 float msg.yaw 偏航速度
     */
    void setGimbalSpeedCallback(const base_interfaces::msg::GimbalPose::SharedPtr msg)
    {
        float pitch_speed = msg->pitch;
        float yaw_speed = msg->yaw;
        GimbalPackage gimbal_package;
        gimbal_package.SetGimbalSpeed(pitch_speed, yaw_speed);
        m_package_manager->send(GIMBAL, gimbal_package);
    }
    /**
     * @brief 设定云台的偏航角速度,俯仰角度
     *
     * @param float msg.pitch 俯仰角度 float msg.yaw 偏航速度
     */
    void setGimbal_YawSpeed_PitchAngle_Callback(const base_interfaces::msg::GimbalPose::SharedPtr msg)
    {
        float pitch_angle = msg->pitch;
        float yaw_speed = msg->yaw;
        GimbalPackage gimbal_package;
        gimbal_package.SetGimbal_YawSpeed_PitchAngle(pitch_angle, yaw_speed);
        m_package_manager->send(GIMBAL, gimbal_package);
    }
    /**
     * @brief 读取云台回传的角度数据
     *
     * @return base_interfaces::msg::GimbalPose GimbalPose；
     */
    void publishGimbalAngle(int index)
    {
        auto msg = base_interfaces::msg::GimbalPose();
        GimbalPackage gimbal_package;

        gimbal_package = m_package_manager->recv<GimbalPackage>(GIMBAL);

        msg.pitch = gimbal_package.m_pitch_angle;
        msg.yaw = gimbal_package.m_yaw_angle;
        msg.roll = 0;
        msg.timestamp = gimbal_package.m_timestamp;

        sensor_msgs::msg::JointState joint_state;
        joint_state.header.stamp = m_node->now();
        joint_state.name.push_back("pitch_joint");
        joint_state.name.push_back("yaw_joint");
        joint_state.position.push_back(gimbal_package.m_pitch_angle);
        joint_state.position.push_back(gimbal_package.m_yaw_angle);

        m_jointStatePublisher -> publish(joint_state);
        publisher<base_interfaces::msg::GimbalPose>(index)->publish(msg);
    }

    /**
     * @brief 发布云台回传的角速度
     * 
     */
    void publishGimbalSpeed(int index)
    {
        auto msg = base_interfaces::msg::GimbalPose();
        GimbalPackage gimbal_package;

        gimbal_package = m_package_manager->recv<GimbalPackage>(GIMBAL);

        msg.pitch = gimbal_package.m_pitch_speed;
        msg.yaw = gimbal_package.m_yaw_speed;
        msg.roll = 0;
        msg.timestamp = gimbal_package.m_timestamp;
        publisher<base_interfaces::msg::GimbalPose>(index)->publish(msg);
    }
    /**
     * @brief 获取扫描信息
     *
     * @return void
     *
     */
    void scanSubscriptionCallback(const base_interfaces::msg::ScanCtrlInfo::SharedPtr msg)
    {
    }
};

} // namespace transport

#endif // __USE_ROS2__

#endif // __GIMBAL_HPP__