#ifndef __GIMBALPOSE_PACKAGE_HPP__
#define __GIMBALPOSE_PACKAGE_HPP__

#include <Package.hpp>

class GimbalPose : public PackageInterFace<GimbalPose>
{
public:
    float pitch;
    float yaw;
    float roll;
public:

    GimbalPose()
    {
        pitch = 0;
        yaw = 0;
        roll = 0;
    }

    GimbalPose(CAN_ID can_id)
    {
        m_can_id = can_id;
        GimbalPose();
    }

    GimbalPose(float pitch, float yaw)
    {
        this->pitch = pitch;
        this->yaw = yaw;
        this->roll = 0;
    }

    GimbalPose(float pitch, float yaw, float roll)
    {
        this->pitch = pitch;
        this->yaw = yaw;
        this->roll = roll;
    }


    GimbalPose(const GimbalPose &gimbalPose)
    {
        pitch = gimbalPose.pitch;
        yaw = gimbalPose.yaw;
        roll = gimbalPose.roll;
    }
    
    GimbalPose decode(Buffer buffer) override
    {
        GimbalPose pose;
        pose.pitch = buffer[0];
        pose.yaw = buffer[1];
        pose.roll = buffer[2];
        return pose;
    }
    
    Buffer encode(GimbalPose gimbalpose) override
    {
        Buffer buffer;
        buffer[0] = gimbalpose.pitch;
        buffer[1] = gimbalpose.yaw;
        return buffer;
    }

};

#endif // __GIMBALPOSE_PACKAGE_HPP__