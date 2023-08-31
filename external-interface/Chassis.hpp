#ifndef __CHASSIS_HPP__
#define __CHASSIS_HPP__

#ifdef __USE_ROS__

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"

#include "BaseROSInterface.hpp"
#include "PackageManager.hpp"
#include "pkg/Chassis.hpp"

#include "base_interfaces/msg/chassis.hpp"
#include "base_interfaces/msg/bt_top.hpp"

class Chassis : public BaseROSInterface
{
public:
    Chassis(const rclcpp::Node::SharedPtr& node, PackageManager::SharedPtr package_manager) : BaseROSInterface(node, package_manager)
    {
        // chassis 控制
        addPublisher<base_interfaces::msg::Chassis>("getSpeed", 10ms, 10, std::bind(&Chassis::publishChassisSpeed, this, 0), this);
        addPublisher<base_interfaces::msg::Chassis>("getTopState", 10ms, 10, std::bind(&Chassis::publishTopState, this, 1), this);
    
        addSubscription<base_interfaces::msg::Chassis>("setChassisSpeed", 10, std::bind(&Chassis::setChassisSpeedCallback, this, _1), this);
        addSubscription<base_interfaces::msg::Chassis>("TopAndSpeed", 10, std::bind(&Chassis::setTopAndSpeedCallback, this, _1), this);
        addSubscription<base_interfaces::msg::Chassis>("top", 10, std::bind(&Chassis::switchTopStateCallback, this, _1), this);
        addSubscription<base_interfaces::msg::BtTop>("BT_Top", 10, std::bind(&Chassis::setTopCallback, this, _1), this);
    }

    /*
     * @brief 设置速度与小陀螺状态
     *
     * @param: bool power 是否开启小陀螺（1为是，0为否）,  float x速度, float y速度，
     */
    void setTopAndSpeedCallback(const base_interfaces::msg::Chassis::SharedPtr msg)
    {
        bool power = msg->power;
        float_t x_speed = msg->x_speed;
        float_t y_speed = msg->y_speed;
        ChassisPackage chassis_package;
        chassis_package.setTopAndSpeed(power, x_speed, y_speed);
        m_package_manager->send(CHASSIS, chassis_package);
    }

    /**
     * @brief 设置小陀螺状态
     *
     * @param base_interfaces::msg::Chassis::SharedPtr msg ,需设置msg->power （1为是，0为否）
     */
    void switchTopStateCallback(const base_interfaces::msg::Chassis::SharedPtr msg)
    {
        bool use_top = msg->power;
        ChassisPackage chassis_package;
        chassis_package.top(use_top);
        m_package_manager->send(CHASSIS, chassis_package);
    }

    void setChassisSpeedCallback(const base_interfaces::msg::Chassis::SharedPtr msg)
    {
        float_t x_speed = msg->x_speed;
        float_t y_speed = msg->y_speed;
        ChassisPackage chassis_package;
        chassis_package.setChassisSpeed(x_speed, y_speed);
        m_package_manager->send(CHASSIS, chassis_package);
    }

    /**
     * @brief 行为树小陀螺接口
     *
     * @param msg
     */
    void setTopCallback(const base_interfaces::msg::BtTop::SharedPtr msg)
    {
        bool use_top = msg->start;

        ChassisPackage chassis_package;
        chassis_package.top(use_top);
        m_package_manager->send(CHASSIS, chassis_package);

        RCLCPP_DEBUG(m_node->get_logger(), "top state: %s", [=]()->std::string
                     {if(use_top) return "open"; 
                      else return "close"; }().c_str());
    }

    /**
     * @brief 发布当前速度
     *
     * 用base_interfaces::msg::Chassis msg 接收，话题名为函数名
     */
    void publishChassisSpeed(int index)
    {
        auto msg = base_interfaces::msg::Chassis();
        auto current_speed = m_package_manager->recv<ChassisPackage>(CHASSIS).getSpeed();
        // if(current_speed.empty())
        //     return;
        msg.power = ChassisPackage::m_top_status;
        msg.x_speed = current_speed[0];
        msg.y_speed = current_speed[1];
        publisher<base_interfaces::msg::Chassis>(index)->publish(msg);
    }

    /**
     * @brief 发布当前小陀螺状态
     *
     * 用 base_interfaces::msg::Chassis msg接收，话题名为函数名
     */
    void publishTopState(int index)
    {
        auto msg = base_interfaces::msg::Chassis();
        auto current_top = m_package_manager->recv<ChassisPackage>(CHASSIS).get_top();
        msg.power = current_top;
        msg.x_speed = 0.0f;
        msg.y_speed = 0.0f;
        publisher<base_interfaces::msg::Chassis>(index)->publish(msg);
    }

};

#endif // __USE_ROS__

#endif // __CHASSIS_HPP__