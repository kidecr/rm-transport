#ifndef __SHOOT_PACKAGE_HPP__
#define __SHOOT_PACKAGE_HPP__

#include "PackageInterface.hpp"

namespace transport{

class ShootPackage : public PackageInterFace<ShootPackage>
{
private:
    uint8_t m_shoot_mode;
    uint8_t m_shoot_num;
    uint16_t m_shoot_rub_speed;     // 播弹轮
    uint16_t m_shoot_boost_speed;   // 摩擦轮

    IENUM BASE_SHOOT_MODE = 0x0F;
    IENUM OPEN_BOX = 0x10;
    static bool m_box_open;
    static bool m_stop_shoot_compulsive;
    static bool m_keep_shoot;

public:
    ShootPackage()
    {
        m_shoot_mode = 0;
        m_shoot_num = 0;
        m_shoot_rub_speed = 0;
        m_shoot_boost_speed = 0;
    }

    ShootPackage(const ShootPackage &shootPackage)
    {
        m_shoot_mode = shootPackage.m_shoot_mode;
        m_shoot_num = shootPackage.m_shoot_num;
        m_shoot_rub_speed = shootPackage.m_shoot_rub_speed;
        m_shoot_boost_speed = shootPackage.m_shoot_boost_speed;
    }


    constexpr ShootPackage &operator=(const ShootPackage &gimbal_pose) = default;

public:
/*********************控制***********************************/
    Buffer encode(ShootPackage shoot) override
    {
        Buffer buffer;
        buffer.resize(8);
        buffer[0] = shoot.m_shoot_mode;
        buffer[1] = shoot.m_shoot_num;
        buffer[2] = shoot.m_shoot_boost_speed >> 8;
        buffer[3] = shoot.m_shoot_boost_speed & 0xff;
        buffer[4] = shoot.m_shoot_rub_speed >> 8;
        buffer[5] = shoot.m_shoot_rub_speed & 0xff;
        return buffer;
    }

    bool openBox()
    {
        m_box_open = !m_box_open;
        m_shoot_mode = OPEN_BOX | BASE_SHOOT_MODE;
        return m_box_open;
    }

    void shootSome(int bullet_num)
    {
        m_shoot_mode = m_box_open ? OPEN_BOX | BASE_SHOOT_MODE : BASE_SHOOT_MODE;
        
        if ((m_stop_shoot_compulsive && bullet_num == 0) || (m_keep_shoot && bullet_num == 0))
            return;
        m_stop_shoot_compulsive = false;
        m_keep_shoot = false;

        m_shoot_rub_speed = 3000;
        m_shoot_boost_speed = 2000;
        m_shoot_num = bullet_num;
    }

    void keepShoot()
    {
        m_shoot_mode = m_box_open ? OPEN_BOX | BASE_SHOOT_MODE : BASE_SHOOT_MODE;
        
        if (!m_keep_shoot)
            m_keep_shoot = true;

        m_stop_shoot_compulsive = false;

        m_shoot_rub_speed = 4500;
        m_shoot_boost_speed = 2000;
        m_shoot_num = 20;
    }

    void stopShoot()
    {
        m_shoot_mode = m_box_open ? OPEN_BOX | BASE_SHOOT_MODE : BASE_SHOOT_MODE;
        
        if (!m_stop_shoot_compulsive)
            m_stop_shoot_compulsive = true;

        m_keep_shoot = false;

        m_shoot_rub_speed = 0;
        m_shoot_boost_speed = 0;
        m_shoot_num = 0;
    }

    void shootNone()
    {
        shootSome(0);
    }

    void shootOnce()
    {
        shootSome(1);
    }

/*********************回传*********************************/
    ShootPackage decode(Buffer buffer) override
    {
        ShootPackage shoot;
        if(buffer.size() < 8) {
            LOGWARN("ShootPackage recv buffer size less than 8");
            return shoot;
        }

        shoot.m_shoot_mode = buffer[0];
        shoot.m_shoot_num = buffer[1];
        shoot.m_shoot_boost_speed = ((uint16_t)buffer[2] << 8) | buffer[3];
        shoot.m_shoot_rub_speed = ((uint16_t)buffer[4] << 8) | buffer[5];
        return shoot;
    }

    int getBulletNum()
    {
        return m_shoot_num;
    }

/********************输出******************************/
    std::string toString() override
    {
        std::stringstream sstream;
        sstream << typeid(*this).name() << std::endl;
        sstream << TO_STR((int)m_shoot_mode) << std::endl;
        sstream << TO_STR((int)m_shoot_num) << std::endl;
        sstream << TO_STR(m_shoot_rub_speed) << std::endl;
        sstream << TO_STR(m_shoot_boost_speed) << std::endl;
        return sstream.str();
    }
};

bool ShootPackage::m_box_open = false;
bool ShootPackage::m_stop_shoot_compulsive = false;
bool ShootPackage::m_keep_shoot = false;

} // namespace transport

#endif // __SHOOT_PACKAGE_HPP__