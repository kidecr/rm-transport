#ifndef __GIMBAL_CONTROL_HPP__
#define __GIMBAL_CONTROL_HPP__

#include "BasePackage.hpp"


#pragma pack(1)
struct Smsg
{
    uint8_t info : 8;
    uint16_t yaw_speed : 12;
    uint16_t pitch_speed : 12;
    uint16_t yaw_angle : 16;
    uint16_t pitch_angle : 16;

    TRANSFORM_FUNC(Smsg)
};
#pragma pack()

class GimbalControl : public PackageInterFace<GimbalControl>
{
public:
    uint8_t info;
    double yaw_speed;
    double pitch_speed;
    double yaw_angle;
    double pitch_angle;
public:
    GimbalControl() = default;
    ~GimbalControl() = default;

    Buffer encode(GimbalControl gimbal_control) override
    { 
        Buffer buffer;
        Smsg msg;
        msg.info = info;
        msg.yaw_speed = buffer_max_interval<12, 3.5, true>(gimbal_control.yaw_speed);
        msg.pitch_speed = buffer_max_interval<12, 3.5, true>(gimbal_control.pitch_speed);
        msg.yaw_angle = buffer_navPI_PI<16>(gimbal_control.yaw_angle);
        msg.pitch_angle = buffer_navPI_PI<16>(gimbal_control.pitch_angle);

        buffer << msg;

        return buffer;
    }

    GimbalControl decode(Buffer buffer) override
    {
        Smsg msg;
        GimbalControl gimbal_control;
        msg << buffer;
        gimbal_control.info = msg.info;
        gimbal_control.yaw_speed = angle_0_2PI<12>(msg.yaw_speed);
        gimbal_control.pitch_speed = angle_0_2PI<12>(msg.pitch_speed);
        gimbal_control.yaw_angle = angle_0_2PI<16>(msg.yaw_angle);
        gimbal_control.pitch_angle = angle_0_2PI<16>(msg.pitch_angle);

        return gimbal_control;
    }

    std::string toString() override
    {
        std::stringstream sstream;
        sstream << typeid(*this).name() << std::endl;
        sstream << TO_STR(yaw_speed) << std::endl;
        sstream << TO_STR(yaw_angle) << std::endl;
        sstream << TO_STR(pitch_speed) << std::endl;
        sstream << TO_STR(pitch_angle) << std::endl;
        return sstream.str();
    }
};


#endif //__GIMBAL_CONTROL_HPP__