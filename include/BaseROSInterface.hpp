#ifndef __BASE_ROS_INTERFACE_HPP__
#define __BASE_ROS_INTERFACE_HPP__

#ifdef __USE_ROS__

#include <chrono>
#include "rclcpp/rclcpp.hpp"

#include "libbase/common.h"

using namespace std::literals::chrono_literals;
using namespace std::placeholders;

struct BaseParam
{
public:
    // 云台控制权限 1是Shooter 2是Navigation 3是Scan
    wmj::GIMBAL_CONTROL_PERMISSION m_gimbalControlPermissions;
    bool m_shoot_enable;
};

template <typename MsgType, typename PeriodType,
          typename QosType, typename CallbackFunc, typename InterfaceName>
void addPublisher(std::string topic_name, PeriodType period, QosType qos, CallbackFunc callback, InterfaceName *p)
{
    auto node = p->m_node;
    auto pub = node->template create_publisher<MsgType>(topic_name, qos);
    p->m_pub_vec.push_back(pub);
    std::function<void()> timer_callback = callback;
    auto pub_timer = node->create_wall_timer(period, timer_callback);
    p->m_pub_timer_vec.push_back(pub_timer);
}

// std::function<void(std::shared_ptr<T>)>
template <typename MsgType, typename QosType, typename CallbackFunc, typename InterfaceName>
void addSubscription(std::string topic_name, QosType qos, CallbackFunc callback, InterfaceName *p)
{
    auto node = p->m_node;
    std::function<void(std::shared_ptr<MsgType>)> sub_callback = callback;
    auto sub = node->template create_subscription<MsgType>(topic_name, qos, sub_callback);
    p->m_sub_vec.push_back(sub);
}

class BaseROSInterface
{

public:
    rclcpp::Node::SharedPtr m_node;
    rclcpp::CallbackGroup::SharedPtr m_callback_group;
    rclcpp::CallbackGroupType m_callback_group_type;
    rclcpp::SubscriptionOptions m_subscription_options;

    std::vector<std::shared_ptr<rclcpp::SubscriptionBase>> m_sub_vec;
    std::vector<std::shared_ptr<rclcpp::PublisherBase>> m_pub_vec;
    std::vector<rclcpp::TimerBase::SharedPtr> m_pub_timer_vec;

    PackageManager::SharedPtr m_package_manager;
    static BaseParam param;

    BaseROSInterface(const rclcpp::Node::SharedPtr &node, PackageManager::SharedPtr package_manager)
    {
        m_node = node;
        m_package_manager = package_manager;
        m_callback_group_type = rclcpp::CallbackGroupType::MutuallyExclusive; // 默认单线程
        m_callback_group = m_node->create_callback_group(m_callback_group_type);

        m_subscription_options = rclcpp::SubscriptionOptions();
        m_subscription_options.callback_group = m_callback_group;
    }

    template <typename MsgType>
    typename rclcpp::Publisher<MsgType>::SharedPtr publisher(int index)
    {
        return std::static_pointer_cast<rclcpp::Publisher<MsgType>>(this->m_pub_vec[index]);
    }
};

BaseParam BaseROSInterface::param;

#endif // __USE_ROS__

#endif // __BASE_ROS_INTERFACE_HPP__