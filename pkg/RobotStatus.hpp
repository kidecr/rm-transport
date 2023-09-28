#ifndef __ROBOT_STATUS_PACKAGE_HPP_
#define __ROBOT_STATUS_PACKAGE_HPP_

#include "PackageInterface.hpp"

namespace transport{

#pragma pack(1)
struct PRobotStatus
{
    uint8_t  game_start : 1;     //0~7 比赛开始结束：1 己方前哨站状态：1 RFID状态（哨兵巡逻区）：1 RFID状态（高地增益）：1
    uint8_t  outpost_state : 1;  //前哨站
    uint8_t  RFID_patrol : 1;    //巡逻区
    uint8_t  RFID_elevated : 1;  //高地RFID，ps：官方高地英语 Elevated Ground
    uint8_t  place_holder_1 : 4;
    uint8_t  armor_id : 3;       //装甲id ： 1～4
    uint8_t  harm_type : 2;      //伤害类型
    uint8_t  place_holder_2 : 1;
    uint16_t sentry_blood : 10;  //哨兵血量
    uint8_t  bullet_remain : 8;  //剩余子弹发射数
    uint16_t outpost_blood : 16; // 前哨站血量
    uint16_t hit_yaw : 16;       //装甲受击yaw方向

    TRANSFORM_FUNC(PRobotStatus)
};
#pragma pack()

class RobotStatusPackage : public PackageInterFace<RobotStatusPackage>
{
public:
    bool m_game_start;        // 比赛开始
    bool m_outpost_state;     // 前哨站
    bool m_RFID_patrol;       // 巡逻区
    bool m_RFID_elevated;     // 高地RFID，ps：官方高地英语 Elevated Ground
    uint8_t m_armor_id;       // 装甲id ： 1～4
    uint8_t m_harm_type;      // 伤害类型
    int m_sentry_blood;       // 哨兵血量
    int m_bullet_remain;      // 剩余子弹发射数
    int m_outpost_blood;      // 前哨站血量
    double m_hit_yaw;         // 装甲受击yaw方向

public:
    RobotStatusPackage()
    {
        m_game_start = false;
        m_outpost_state = false;
        m_RFID_patrol = false;
        m_RFID_elevated = false;
        m_armor_id = 0;
        m_harm_type = 0;
        m_sentry_blood = 0;
        m_bullet_remain = 0;
        m_outpost_blood = 0;
        m_hit_yaw = 0;
    }

    RobotStatusPackage(const RobotStatusPackage &robot_status_package)
    {
        m_game_start        = robot_status_package.m_game_start;   
        m_outpost_state     = robot_status_package.m_outpost_state;
        m_RFID_patrol         = robot_status_package.m_RFID_patrol;    
        m_RFID_elevated       = robot_status_package.m_RFID_elevated;  
        m_armor_id            = robot_status_package.m_armor_id;       
        m_harm_type           = robot_status_package.m_harm_type;      
        m_sentry_blood        = robot_status_package.m_sentry_blood;   
        m_bullet_remain       = robot_status_package.m_bullet_remain;  
        m_outpost_blood       = robot_status_package.m_outpost_blood;  
        m_hit_yaw             = robot_status_package.m_hit_yaw;        
    }

    constexpr RobotStatusPackage &operator=(const RobotStatusPackage &robot_status_package) = default;

public:

    void encode(RobotStatusPackage &robot_status_package, Buffer &buffer) override
    {
        buffer.resize(sizeof(PRobotStatus));
        PRobotStatus robot_status;
        robot_status.game_start     = robot_status_package.m_game_start;
        robot_status.outpost_state  = robot_status_package.m_outpost_state;
        robot_status.RFID_patrol    = robot_status_package.m_RFID_patrol;
        robot_status.RFID_elevated  = robot_status_package.m_RFID_elevated;
        robot_status.armor_id       = robot_status_package.m_armor_id;
        robot_status.harm_type      = robot_status_package.m_harm_type;
        robot_status.sentry_blood   = robot_status_package.m_sentry_blood;
        robot_status.bullet_remain  = robot_status_package.m_bullet_remain / 3;
        robot_status.outpost_blood  = robot_status_package.m_outpost_blood;
        robot_status.hit_yaw        = buffer_0_2PI<16>(robot_status_package.m_hit_yaw);
        buffer << robot_status;
        return;
    }

    void decode(RobotStatusPackage &robot_status_package, Buffer &buffer) override
    {
        if(buffer.size() < 8) {
            LOGWARN("RobotStatusPackage recv buffer size less than 8");
            return;
        }
        
        robot_status_package.m_game_start    = robot_status.game_start;     
        robot_status_package.m_outpost_state = robot_status.outpost_state;  
        robot_status_package.m_RFID_patrol   = robot_status.RFID_patrol;    
        robot_status_package.m_RFID_elevated = robot_status.RFID_elevated;  
        robot_status_package.m_armor_id      = robot_status.armor_id;       
        robot_status_package.m_harm_type     = robot_status.harm_type;      
        robot_status_package.m_sentry_blood  = robot_status.sentry_blood;   
        robot_status_package.m_bullet_remain = robot_status.bullet_remain * 3;  
        robot_status_package.m_outpost_blood = robot_status.outpost_blood;  
        robot_status_package.m_hit_yaw       = angle_0_2PI<16>(robot_status.hit_yaw);        

        return;
    }
};

} // namespace transport

#endif // __ROBOT_STATUS_PACKAGE_HPP_