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

    constexpr GimbalPose &operator=(const GimbalPose &gimbal_pose) = default;

    GimbalPose decode(Buffer buffer) override
    {
        GimbalPose pose;
        if(buffer.size() < 8) return pose;
        pose.pitch = buffer[0];
        pose.yaw = buffer[1];
        pose.roll = buffer[2];
        return pose;
    }

    Buffer encode(GimbalPose gimbalpose) override
    {
        Buffer buffer;
        buffer.resize(8);
        buffer[0] = gimbalpose.pitch;
        buffer[1] = gimbalpose.yaw;
        buffer[2] = gimbalpose.roll;
        return buffer;
    }

    std::string toString() override
    {
        std::stringstream sstream;
        sstream << typeid(*this).name() << std::endl;
        sstream << TO_STR(yaw) << std::endl;
        sstream << TO_STR(pitch) << std::endl;
        sstream << TO_STR(roll) << std::endl;
        return sstream.str();
    }
};

#endif // __GIMBALPOSE_PACKAGE_HPP__