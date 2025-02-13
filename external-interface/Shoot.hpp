#ifndef __SHOOT_HPP__
#define __SHOOT_HPP__

#ifdef __USE_ROS2__

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"

#include "impls/BaseROSInterface.hpp"
#include "impls/PackageID.hpp"
#include "PackageManager.hpp"

#include "protocal/GlobalParam.hpp"
#include "protocal/Protocal.hpp"

#include "pkg/Shoot.hpp"
// #include "pkg/MainControl.hpp"

#include "transport/msg/shooter.hpp"

namespace transport{

class Shoot : public BaseROSInterface
{
public:
    Shoot(const rclcpp::Node::SharedPtr& node, PackageManager::SharedPtr package_manager) : BaseROSInterface(node, package_manager)
    {
        addPublisher<transport::msg::Shooter>("GetBulletNumber", 100ms, 10, std::bind(&Shoot::getBulletNumberCallback, this, 0), this);
        // addPublisher<transport::msg::Shooter>("HighShootSpeed", 100ms, 10, std::bind(&Shoot::highShootSpeedCallback, this, 1), this);
        
        addSubscription<transport::msg::Shooter>("ShootSome", 10, std::bind(&Shoot::shootSomeCallback, this, std::placeholders::_1), this);
        addSubscription<transport::msg::Shooter>("StopShoot", 10, std::bind(&Shoot::stopShootCallback, this, std::placeholders::_1), this);
        addSubscription<transport::msg::Shooter>("openBox", 10, std::bind(&Shoot::openBoxCallback, this, std::placeholders::_1), this);
        // addSubscription<transport::msg::BtAimer>("BT_shooter", 10, std::bind(&Shoot::shootControlCallback, this, std::placeholders::_1), this);
    }

    /**
     *@brief: 连发子弹
     *
     *@param:transport::msg::Shooter int msg.bulletnum发射数 float 子弹射速 float 拨弹轮转速
     * 子弹射速设置目前无效，可以在control_base/src下的源文件中改
     * 射击子弹数量 0为不射击(开摩擦轮)，n为射击n发(n>0)，-1为持续射击（此时持续发射，给0才会停）-2为解释能
     */
    void shootSomeCallback(const transport::msg::Shooter::SharedPtr msg)
    {
        int bulletnum = msg->bulletnum;
        ShootPackage shoot_package;
        if (bulletnum == -2)
        {
            shoot_package.stopShoot();
        }
        if (bulletnum == -1)
        {
            shoot_package.keepShoot();
        }
        else if (bulletnum == 0)
        {
            shoot_package.shootSome(0);
        }
        else if (bulletnum > 0)
        {
            shoot_package.shootSome(1);
        }
        if(bulletnum != 0)
            m_package_manager->send(CAN_ID_SHOOT, shoot_package);
    }

    /**
     *@brief: 停止发射，包括摩擦轮 ,transport::msg::Shooter msg; bool msg.stop1停,0不处理
     **/
    void stopShootCallback(const transport::msg::Shooter::SharedPtr msg)
    {
        auto stop = msg->stop;
        if (stop == 1)
        {
            ShootPackage shoot_package;
            shoot_package.stopShoot();
            m_package_manager->send(CAN_ID_SHOOT, shoot_package);
        }
    }

    /**
     *@brief: 开弹仓 transport::msg::Shooter msg; bool msg.box;1开0关
     */
    void openBoxCallback(const transport::msg::Shooter::SharedPtr msg)
    {
        auto box = msg->box;
        if (box == 1)
        {
            ShootPackage shoot_package;
            shoot_package.openBox();
            m_package_manager->send(CAN_ID_SHOOT, shoot_package);
        }
    }

    // /**
    //  *@brief: 是否拉高射速
    //  *
    //  *@return:transport::msg::Shooter msg, bool msg.shoot_mode是否拉高
    //  *1为高射速模式，2为低射速模式（从电控获取状态）
    //  *
    //  */
    // void highShootSpeedCallback(int index)
    // {
    //     auto main_control_package = m_package_manager->recv<MainControlPackage>(MAIN_CONTROL);
    //     auto msg = transport::msg::Shooter();
    //     msg.shoot_mode = main_control_package.HighShootSpeed();
    //     publisher<transport::msg::Shooter>(index)->publish(msg);
    // }

    /**
     * @brief 获取连发剩余子弹数
     *
     * @return int msg.getbulletnumber
     */
    void getBulletNumberCallback(int index)
    {
        int bullet_number = m_package_manager->recv<ShootPackage>(CAN_ID_SHOOT).getBulletNum();
        auto msg = transport::msg::Shooter();
        msg.getbulletnumber = bullet_number;
        publisher<transport::msg::Shooter>(index)->publish(msg);
    }

    //! !!!!!!!!!!!!!BtAimer现在用来控制发射权限，当bullet_rate=1时，给发射权限
    // /**
    //  * @brief
    //  *
    //  * @param msg
    //  */
    // void shootControlCallback(const transport::msg::BtAimer::SharedPtr msg)
    // {
    //     LOGDEBUG("bullet rate: %d", msg->bullet);
    // }
};

} // namespace transport

#endif // __USE_ROS2__

#endif // __SHOOT_HPP__