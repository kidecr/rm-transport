#ifndef __GIMBAL_PACKAGE_HPP__
#define __GIMBAL_PACKAGE_HPP__

#include "impls/PackageInterface.hpp"

#ifdef __USE_LIBBASE__
#include "libbase/common.h"
#endif // __USE_LIBBASE__

namespace transport{

#pragma pack(1)
struct PGimbal
{
    uint8_t info : 8;
    uint16_t yaw_speed : 12;
    uint16_t pitch_speed : 12;
    uint16_t yaw_angle : 16;
    uint16_t pitch_angle : 16;

    TRANSFORM_FUNC(PGimbal)
};

struct PGyro
{
    uint16_t other : 16;
    uint16_t yaw : 16;
    uint16_t pitch : 16;
    uint16_t roll : 16;

    TRANSFORM_FUNC(PGyro)
};
#pragma pack()


class GimbalPackage : public PackageInterFace<GimbalPackage>
{
public:
    double m_pitch_speed;
    double m_yaw_speed;
    double m_pitch_angle;
    double m_yaw_angle;
    double m_timestamp;
private:
    uint8_t mode;

    UENUM BASE_COOR = 0x00; // 云台系
    UENUM GYRO_COOR = 0x40; // 地面系
    UENUM SET_ANGLE = 0x33; // setGimbalAngle
    UENUM SET_SPEED = 0x0f; // setGimbalSpeed
    UENUM SET_YS_PA = 0x27; // SetGimbal_YawSpeed_PitchAngle
    static bool m_coor; // true 地面系， false 云台系
public:
    GimbalPackage()
    {
        m_pitch_speed = 0.0;
        m_pitch_angle = 0.0;
        m_yaw_speed = 0.0;
        m_yaw_angle = 0.0;
        mode = 0;
    }

    GimbalPackage(const GimbalPackage &gimbal_package)
    {
        m_pitch_angle = gimbal_package.m_pitch_angle;
        m_pitch_speed = gimbal_package.m_pitch_speed;
        m_yaw_angle   = gimbal_package.m_yaw_angle;
        m_yaw_speed   = gimbal_package.m_yaw_speed;
        mode          = gimbal_package.mode;
    }

    constexpr GimbalPackage &operator=(const GimbalPackage &gimbal_package) = default;
public:

    void encode(GimbalPackage &gimbal_package, Buffer &buffer) override
    {
        buffer.resize(8);
        PGimbal gimbal;

        gimbal.info         = gimbal_package.mode | (m_coor ? GYRO_COOR : BASE_COOR);
        gimbal.yaw_speed    = buffer_navPI_PI<12>(gimbal_package.m_yaw_speed);
        gimbal.pitch_speed  = buffer_navPI_PI<12>(gimbal_package.m_pitch_speed);
        gimbal.yaw_angle    = buffer_0_2PI<16>(gimbal_package.m_yaw_angle);
        gimbal.pitch_angle  = buffer_0_2PI<16>(gimbal_package.m_pitch_angle);
        
        buffer << gimbal;
        return;
    }

    void decode(GimbalPackage &gimbal_package, Buffer &buffer) override
    {
        if(buffer.size() < 8) {
            LOGWARN("GimbalPackage recv buffer size less than 8");
            return;
        }
        
        if(m_coor) // 地面系
        {
            decodeByPGyro(gimbal_package, buffer);
        }
        else // 云台系 
        {
#ifndef Sentry // 哨兵和步兵解析的包不一样
            decodeByPGimbal(gimbal_package, buffer);
#else 
            decodeByPGyro(gimbal_package, buffer);
#endif // Sentry
        }

        return;
    }

    // 云台系回传， 有速度有角度
    void decodeByPGimbal(GimbalPackage &gimbal_package, Buffer &buffer)
    {
        PGimbal gimbal;
        gimbal << buffer;
        gimbal_package.m_yaw_speed = angle_navPI_PI<12>(gimbal.yaw_speed);
        gimbal_package.m_pitch_speed = angle_navPI_PI<12>(gimbal.pitch_speed);
        gimbal_package.m_yaw_angle = angle_0_2PI<16>(gimbal.yaw_angle);
        gimbal_package.m_pitch_angle = angle_0_2PI<16>(gimbal.pitch_angle);
        return;
    }
    // 车体系回传，只有y p r角度，其中roll弃用
    void decodeByPGyro(GimbalPackage &gimbal_package, Buffer &buffer)
    {
        PGyro gyro;
        gyro << buffer;
        gimbal_package.m_timestamp = (double)gyro.other;
        gimbal_package.m_yaw_angle = angle_0_2PI<16>(gyro.yaw);
        gimbal_package.m_pitch_angle = angle_0_2PI<16>(gyro.pitch);
        // roll不解析
        return;
    }

    std::string toString() override
    {
        std::stringstream ss;
        ss << "{ " << __CLASS__ ;
        ss << " mode: " << (int)mode;
        ss << " yaw_speed: " << m_yaw_speed;
        ss << " pitch_speed: " << m_pitch_speed;
        ss << " yaw_angle: " << m_yaw_angle;
        ss << " pitch_angle: " << m_pitch_angle << "}";
        return ss.str();
    }

/***********************接口**************************************************/
    /**
     * @brief 使云台以最高速度转到目标角度
     *
     * @param float 俯仰角度 float 偏航角度
     */
    void SetGimbalAngle(float pitch_angle, float yaw_angle)
    {
        mode = SET_ANGLE;
        m_pitch_angle = pitch_angle;
        m_yaw_angle = yaw_angle;
    }

    /**
     * @brief 设定云台的角速度
     *
     * @param float 俯仰速度 float 偏航速度
     */
    void SetGimbalSpeed(float pitch_speed, float yaw_speed)
    {
        mode = SET_SPEED;
        m_pitch_speed = pitch_speed;
        m_yaw_speed = yaw_speed;
    }

    /**
     * @brief 设定云台的偏航角速度,俯仰角度
     *
     * @param float 俯仰角度 float 偏航速度
     */
    void SetGimbal_YawSpeed_PitchAngle(float pitch_angle, float yaw_speed)
    {
        mode = SET_YS_PA;
        m_pitch_angle = pitch_angle;
        m_yaw_speed = yaw_speed;
    }
#ifdef __USE_LIBBASE__
    /**
     * @brief 读取云台回传的角度数据
     *
     * @return GimbalPose
     */
    wmj::GimbalPose GetGimbalAngle()
    {
        return wmj::GimbalPose(m_pitch_angle, m_yaw_angle, 0.0, m_timestamp);
    }

    /**
     * @brief 获取云台当前角速度值
     *
     * @return
     */
    wmj::GimbalPose GetGimbalSpeed()
    {
        return wmj::GimbalPose(m_pitch_speed, m_yaw_speed, 0.0, 0.0);
    }
#endif // __USE_LIBBASE__
    /**
     * @brief 切换坐标系
     * 
     * @param coor 目标坐标系 true：地面系 false：云台系
     * @return bool 当前坐标系
     */
    bool switchCoor(bool coor)
    {
        m_coor = coor;
        return m_coor;
    }

    /**
     * @brief 获取当前坐标系
     * 
     * @return true 地面系
     * @return false 云台系
     */
    bool getCoor()
    {
        return m_coor;
    }
};

bool GimbalPackage::m_coor = false;

} // namespace transport

#endif // __GIMBAL_PACKAGE_HPP__