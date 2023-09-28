#ifndef __CHASSIS_PACKAGE_HPP__
#define __CHASSIS_PACKAGE_HPP__

#include "PackageInterface.hpp"

namespace transport{

#pragma pack(1)
struct PChassis
{
    union {
        struct{
            uint8_t open_top : 1;
            uint8_t x_speed_available : 1;
            uint8_t y_speed_available : 1;
            uint8_t place_holder : 5;
        } control;
        struct {
            uint8_t motor_online : 4;
            uint8_t gyroscope_online : 1;
            uint8_t magnetometer_online : 1;
            uint8_t top_status : 1;
            uint8_t place_holder : 1;
        } feedback;
    } info;

    uint16_t x_speed : 16;
    uint16_t y_speed : 16;

    TRANSFORM_FUNC(PChassis)
};
#pragma pack()

class ChassisPackage : public PackageInterFace<ChassisPackage>
{
public:
    bool m_top;
    double m_x_speed;
    double m_y_speed;
    static bool m_top_status;
public:
    ChassisPackage()
    {
        m_top = false;
        m_x_speed = 0.0;
        m_y_speed = 0.0;
    }

    ChassisPackage(const ChassisPackage &chassis_package)
    {
        m_top = chassis_package.m_top;
        m_x_speed = chassis_package.m_x_speed;
        m_y_speed = chassis_package.m_y_speed;  
    }

    constexpr ChassisPackage &operator=(const ChassisPackage &chassis_package) = default;

public:
/*********************************编码****************************************/
    void encode(ChassisPackage &chassis_package, Buffer &buffer) override
    {
        buffer.resize(sizeof(PChassis));
        PChassis chassis;

        chassis.info.control.open_top = chassis_package.m_top;
        chassis.info.control.x_speed_available = true;
        chassis.info.control.y_speed_available = true;
        chassis.x_speed = buffer_max_interval<16, 3.5, false>(chassis_package.m_x_speed);
        chassis.y_speed = buffer_max_interval<16, 3.5, false>(chassis_package.m_y_speed);

        buffer << chassis;
        return;
    }

    /*
     * @brief 设置速度与小陀螺状态
     *
     * @param: bool 是否开启小陀螺（1为是，0为否）,  float x速度, float y速度，
     */
    void setTopAndSpeed(bool power, float x_speed, float y_speed)
    {
        m_top_status = power;
        m_top = power;
        m_x_speed = x_speed;
        m_y_speed = y_speed;
    }

    void setChassisSpeed(float x_speed, float y_speed)
    {
        m_x_speed = x_speed;
        m_y_speed = y_speed;
    }

    /*
     * @brief  小陀螺开关
     *
     * @param: bool  是否开启小陀螺（1为是，0为否）
     */
    void top(bool power)
    {
        m_top_status = power;
        m_top = power;
    }
    /*
     * @brief 刹车
     */
    void brake()
    {
        m_top = false;
        m_top_status = false;
        m_x_speed = 0.0;
        m_y_speed = 0.0;
    }

/*********************************解码**************************************/
    void decode(ChassisPackage &chassis_package, Buffer &buffer) override
    {
        if(buffer.size() < 8) {
            LOGWARN("ChassisPackage recv buffer size less than 8");
            return;
        }
        
        PChassis chassis;
        chassis << buffer;
        chassis_package.m_top = chassis.info.feedback.top_status;
        chassis_package.m_x_speed = angle_max_interval<16, 3.5, false>(chassis.x_speed);
        chassis_package.m_y_speed = angle_max_interval<16, 3.5, false>(chassis.y_speed);

        return;
    }

    /*
     * @brief  查看当前速度
     *
     * 返回std::vector<float>  x_speed ,y_speed;
     */
    std::vector<float> getSpeed()
    {
        m_top_status = m_top;
        return std::vector<float>{(float)m_x_speed, (float)m_y_speed};
    }
    /*
     * @brief  查看是否有小陀螺
     *
     * 返回1为开启，0为未开启
     */
    bool get_top()
    {
        m_top_status = m_top;
        return m_top;
    }

};

bool ChassisPackage::m_top_status = false;

} // namespace transport
#endif // __CHASSIS_PACKAGE_HPP__