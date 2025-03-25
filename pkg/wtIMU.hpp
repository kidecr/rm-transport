#ifndef __WTIMU_PACKAGE_HPP__
#define __WTIMU_PACKAGE_HPP__

#include "impls/PackageInterface.hpp"

namespace transport{

#pragma pack(1)
struct PWTIMU
{
    uint8_t head;
    uint8_t flag;
    uint8_t AXL;
    uint8_t AXH;
    uint8_t AYL;
    uint8_t AYH;
    uint8_t AZL;
    uint8_t AZH;
    uint8_t WXL;
    uint8_t WXH;
    uint8_t WYL;
    uint8_t WYH;
    uint8_t WZL;
    uint8_t WZH;
    uint8_t RollL;
    uint8_t RollH;
    uint8_t PitchL;
    uint8_t PitchH;
    uint8_t YawL;
    uint8_t YawH;

    TRANSFORM_FUNC(PWTIMU)
};
#pragma pack()

class WTIMU : public PackageInterFace<WTIMU>
{
private:
    constexpr static double G = 9.8; // m/s^2
public:
    // 加速度
    double m_ax, m_ay, m_az;
    // 角速度
    double m_wx, m_wy, m_wz;
    // 欧拉角
    double m_roll, m_pitch, m_yaw;

public:
    WTIMU():
        m_ax(0), m_ay(0), m_az(0), 
        m_wx(0), m_wy(0), m_wz(0), 
        m_roll(0), m_pitch(0), m_yaw(0)
    {}

    WTIMU(const WTIMU &shootPackage):
        m_ax(shootPackage.m_ax), m_ay(shootPackage.m_ay), m_az(shootPackage.m_az), 
        m_wx(shootPackage.m_wx), m_wy(shootPackage.m_wy), m_wz(shootPackage.m_wz),
        m_roll(shootPackage.m_roll), m_pitch(shootPackage.m_pitch), m_yaw(shootPackage.m_yaw)
    {}


    constexpr WTIMU &operator=(const WTIMU &gimbal_pose) = default;

public:
/*********************控制***********************************/
    void encode(WTIMU &shoot, Buffer &buffer) override
    {
        return;
    }

/*********************回传*********************************/
    void decode(WTIMU &wt_imu, Buffer &buffer) override
    {
        LOGDEBUG("check buffer size %d and imu size %d", buffer.size(), sizeof(PWTIMU));
        if(buffer.size() < sizeof(PWTIMU)) {
            LOGWARN("WTIMUPackage recv buffer size less than 8");
            return;
        }
        PWTIMU imu;
        imu << buffer;
        wt_imu.m_ax = (imu.AXL | (imu.AXH << 8)) / 32768.0 * 16 * G; // m/s^2
        wt_imu.m_ay = (imu.AYL | (imu.AYH << 8)) / 32768.0 * 16 * G; // m/s^2
        wt_imu.m_az = (imu.AZL | (imu.AZH << 8)) / 32768.0 * 16 * G; // m/s^2

        wt_imu.m_wx = (imu.WXL | (imu.WXH << 8)) / 32768.0 * 2000; // °/s
        wt_imu.m_wy = (imu.WYL | (imu.WYH << 8)) / 32768.0 * 2000; // °/s
        wt_imu.m_wz = (imu.WZL | (imu.WZH << 8)) / 32768.0 * 2000; // °/s

        wt_imu.m_roll = (imu.RollL | (imu.RollH << 8)) / 32768.0 * 180; // °
        wt_imu.m_pitch = (imu.PitchL | (imu.PitchH << 8)) / 32768.0 * 180; // °
        wt_imu.m_yaw = (imu.YawL | (imu.YawH << 8)) / 32768.0 * 180; // °

        return;
    }

/********************输出******************************/
    std::string toString() override
    {
        std::stringstream sstream;
        sstream << __CLASS__ << std::endl;
        sstream << TO_STR(m_ax) << std::endl;
        sstream << TO_STR(m_ay) << std::endl;
        sstream << TO_STR(m_az) << std::endl;

        sstream << TO_STR(m_wx) << std::endl;
        sstream << TO_STR(m_wy) << std::endl;
        sstream << TO_STR(m_wz) << std::endl;

        sstream << TO_STR(m_roll) << std::endl;
        sstream << TO_STR(m_pitch) << std::endl;
        sstream << TO_STR(m_yaw) << std::endl;

        return sstream.str();
    }
};

} // namespace transport

#endif // __WTIMU_PACKAGE_HPP__