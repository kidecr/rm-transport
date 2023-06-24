#ifndef __GIMBAL_GLOBAL_LINK__
#define __GIMBAL_GLOBAL_LINK__

#include "Package.hpp"

#pragma pack(1)
struct SGimbalGlobalLink
{
    uint16_t time_stamp : 16;
    uint16_t yaw_angle : 16;
    uint16_t pitch_angle : 16;
    uint16_t roll_angle : 16;

    TRANSFORM_FUNC(SGimbalGlobalLink)
};
#pragma pack()

class GimbalGlobalLink : public PackageInterFace<GimbalGlobalLink>
{
public:
    uint16_t time_stamp;
    double yaw_angle;
    double pitch_angle;
    double roll_angle;
public:
    GimbalGlobalLink() = default;
    ~GimbalGlobalLink() = default;

    GimbalGlobalLink decode(Buffer buffer) override
    {
        GimbalGlobalLink gl;
        SGimbalGlobalLink sgl;
        sgl << buffer;

        gl.time_stamp = sgl.time_stamp;
        gl.pitch_angle = angle_0_2PI<16>(sgl.pitch_angle);
        gl.yaw_angle = angle_0_2PI<16>(sgl.yaw_angle);
        gl.roll_angle = angle_0_2PI<16>(sgl.roll_angle);

        return gl;
    }

    Buffer encode(GimbalGlobalLink target) override
    {
        Buffer buffer;
        SGimbalGlobalLink sgl;
        sgl.time_stamp = target.time_stamp;
        sgl.pitch_angle = buffer_0_2PI<16>(target.pitch_angle);
        sgl.yaw_angle = buffer_0_2PI<16>(target.pitch_angle);
        sgl.roll_angle = buffer_0_2PI<16>(target.roll_angle);

        buffer << sgl;
        return buffer;
    }
};


#endif // __GIMBAL_BASE_LINK__