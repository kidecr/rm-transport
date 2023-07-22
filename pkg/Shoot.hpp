#ifndef __SHOOT_PACKAGE_HPP__
#define __SHOOT_PACKAGE_HPP__

#include <Package.hpp>

class ShootPackage : public PackageInterFace<ShootPackage>
{
public:
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

    constexpr ShootPackage &operator=(const ShootPackage &gimbal_pose) = default;

    ShootPackage decode(Buffer buffer) override
    {
        ShootPackage shoot;
        if(buffer.size() < 8) 
            return shoot;

        shoot.shoot_mode = buffer[0];
        shoot.shoot_num = buffer[1];
        shoot.shoot_boost_speed = ((uint16_t)buffer[2] << 8) | buffer[3];
        shoot.shoot_rub_speed = ((uint16_t)buffer[4] << 8) | buffer[5];
        return shoot;
    }

    Buffer encode(ShootPackage shoot) override
    {
        Buffer buffer;
        buffer.resize(8);
        buffer[0] = shoot.shoot_mode;
        buffer[1] = shoot.shoot_num;
        buffer[2] = shoot.shoot_boost_speed >> 8;
        buffer[3] = shoot.shoot_boost_speed & 0xff;
        buffer[4] = shoot.shoot_rub_speed >> 8;
        buffer[5] = shoot.shoot_rub_speed & 0xff;
        return buffer;
    }

    std::string toString() override
    {
        std::stringstream sstream;
        sstream << typeid(*this).name() << std::endl;
        sstream << TO_STR((int)shoot_mode) << std::endl;
        sstream << TO_STR((int)shoot_num) << std::endl;
        sstream << TO_STR(shoot_rub_speed) << std::endl;
        sstream << TO_STR(shoot_boost_speed) << std::endl;
        return sstream.str();
    }
};

#endif // __SHOOT_PACKAGE_HPP__