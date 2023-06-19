#ifndef __SHOOT_PACKAGE_HPP__
#define __SHOOT_PACKAGE_HPP__

#include <Package.hpp>

class ShootPackage : public PackageInterFace<ShootPackage>
{
private:
    uint8_t shoot_mode;
    uint8_t shoot_num;
    uint16_t shoot_rub_speed;
    uint16_t shoot_boost_speed;

public:
    ShootPackage()
    {
        shoot_mode = 0;
        shoot_num = 0;
        shoot_rub_speed = 0;
        shoot_boost_speed = 0;
    }

    ShootPackage(const ShootPackage &shootPackage)
    {
        shoot_mode = shootPackage.shoot_mode;
        shoot_num = shootPackage.shoot_num;
        shoot_rub_speed = shootPackage.shoot_rub_speed;
        shoot_boost_speed = shootPackage.shoot_boost_speed;
    }

    ShootPackage decode(Buffer buffer) override
    {
        ShootPackage shoot;
        shoot.shoot_mode = buffer[0] & 0x03;
        return shoot;
    }

    Buffer encode(ShootPackage shoot) override
    {
        Buffer buffer;
        buffer[0] = shoot.shoot_num;
        buffer[1] = 10;
        return buffer;
    }
};

#endif // __SHOOT_PACKAGE_HPP__