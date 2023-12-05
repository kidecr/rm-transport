#ifndef __BASE_ROS_INTERFACE_HPP__
#define __BASE_ROS_INTERFACE_HPP__

#ifdef __USE_ROS2__

#include <chrono>
#include "rclcpp/rclcpp.hpp"

#include "libbase/common.h"

using namespace std::literals::chrono_literals;
using namespace std::placeholders;

namespace transport{

class Param
{
public:
    // 云台控制权限 1是Shooter 2是Navigation 3是Scan
    wmj::GIMBAL_CONTROL_PERMISSION m_gimbalControlPermissions;
    bool m_shoot_enable;
private:
    Param() = default;
    static Param* m_param;
    static std::mutex m_lock;
public:
    static Param* param()
    {
        if(m_param == NULL)
        {
            std::lock_guard<std::mutex> lock(m_lock);
            if(m_param == NULL)
                m_param = new Param();
        }
        return m_param;
    }
};

Param* Param::m_param = NULL;
std::mutex Param::m_lock;

/**
 * @brief 创建publisher和timer
 * 
 * @tparam MsgType 消息类型
 * @tparam PeriodType 
 * @tparam QosType 
 * @tparam CallbackFunc 
 * @tparam InterfaceName 
 * @param topic_name topic名
 * @param period 发布周期
 * @param qos qos
 * @param callback 回调函数 
 * @param p 类指针，指向要添加publisher的目标类
 */
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

/**
 * @brief 创建subscription, 默认使用参数p中的options
 * 
 * @tparam MsgType 消息类型
 * @tparam QosType 
 * @tparam CallbackFunc 
 * @tparam InterfaceName 
 * @param topic_name topic名
 * @param qos qos
 * @param callback 回调函数
 * @param p 类指针，指向要添加subscription的目标类
 */
template <typename MsgType, typename QosType, typename CallbackFunc, typename InterfaceName>
void addSubscription(std::string topic_name, QosType qos, CallbackFunc callback, InterfaceName *p)
{
    auto node = p->m_node;
    std::function<void(std::shared_ptr<MsgType>)> sub_callback = callback;
    auto sub = node->template create_subscription<MsgType>(topic_name, qos, sub_callback, p->m_subscription_options);
    p->m_sub_vec.push_back(sub);
}

/**
 * @brief 创建subscription, 使用指定options
 * 
 * @tparam MsgType 消息类型
 * @tparam QosType 
 * @tparam CallbackFunc 
 * @tparam InterfaceName 
 * @param topic_name topic名
 * @param qos qos
 * @param callback 回调函数
 * @param p 类指针，指向要添加subscription的目标类
 * @param subscription_options 指定SubscriptionOption
 */
template <typename MsgType, typename QosType, typename CallbackFunc, typename InterfaceName>
void addSubscription(std::string topic_name, QosType qos, CallbackFunc callback, InterfaceName *p, rclcpp::SubscriptionOptions subscription_options)
{
    auto node = p->m_node;
    std::function<void(std::shared_ptr<MsgType>)> sub_callback = callback;
    auto sub = node->template create_subscription<MsgType>(topic_name, qos, sub_callback, subscription_options);
    p->m_sub_vec.push_back(sub);
}

/**
 * @brief 创建定时器,使用参数p中默认的回调组
 * 
 * @tparam PeriodType 
 * @tparam CallbackFunc 
 * @tparam InterfaceName 
 * @param period 周期
 * @param callback 回调函数
 * @param p 类指针，指向要添加timer的目标类
 */
template <typename PeriodType, typename CallbackFunc, typename InterfaceName>
void addTimer(PeriodType period, CallbackFunc callback, InterfaceName *p)
{
    auto node = p->m_node;
    std::function<void()> timer_callback = callback;
    auto pub_timer = node->create_wall_timer(period, timer_callback, p->m_callback_group);
    p->m_pub_timer_vec.push_back(pub_timer);
}

/**
 * @brief 创建定时器,使用参数中指定的回调组
 * 
 * @tparam PeriodType 
 * @tparam CallbackFunc 
 * @tparam InterfaceName 
 * @param period 周期
 * @param callback 回调函数
 * @param p 类指针，指向要添加timer的目标类
 * @param callback_group 回调组
 */
template <typename PeriodType, typename CallbackFunc, typename InterfaceName>
void addTimer(PeriodType period, CallbackFunc callback, InterfaceName *p, rclcpp::CallbackGroup::SharedPtr callback_group)
{
    auto node = p->m_node;
    std::function<void()> timer_callback = callback;
    auto pub_timer = node->create_wall_timer(period, timer_callback, callback_group);
    p->m_pub_timer_vec.push_back(pub_timer);
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

    BaseROSInterface(const rclcpp::Node::SharedPtr &node, PackageManager::SharedPtr package_manager)
    : m_subscription_options(),
    m_node(node),
    m_package_manager(package_manager)
    {
        m_callback_group_type = rclcpp::CallbackGroupType::MutuallyExclusive; // 默认单线程
        m_callback_group = m_node->create_callback_group(m_callback_group_type);

        m_subscription_options.callback_group = m_callback_group;
    }

    template <typename MsgType>
    typename rclcpp::Publisher<MsgType>::SharedPtr publisher(int index)
    {
        return std::static_pointer_cast<rclcpp::Publisher<MsgType>>(this->m_pub_vec[index]);
    }
};

} // namespace transport

#endif // __USE_ROS2__

#endif // __BASE_ROS_INTERFACE_HPP__