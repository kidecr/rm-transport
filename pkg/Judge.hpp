#ifndef __JUDGE_PACKAGE_HPP__
#define __JUDGE_PACKAGE_HPP__

#include "PackageInterface.hpp"

class JudgePackage : public PackageInterFace<JudgePackage>
{
private:
    uint8_t m_rune_status;
    uint8_t m_shoot_speed_level;
    int m_last_shoot_speed;
    int m_remain_time;

    static bool m_shoot_speed_initialized;
    static ROBO_SHOOT m_shoot_level;
    static std::deque<float> m_prev_shoot_speed;
    static float m_cur_shoot_speed;
    static float m_fifteen_shoot_speed;
    static float m_eighteen_shoot_speed;
    static float m_thirty_shoot_speed; 
    static bool m_clear_shoot_speed;
public:
    JudgePackage()
    {
        m_rune_status = 0;
        m_shoot_speed_level = 0;
        m_last_shoot_speed = 0;
        m_remain_time = 0;
    }

    JudgePackage(const JudgePackage &judge_package)
    {
        m_rune_status = judge_package.m_rune_status;
        m_shoot_speed_level = judge_package.m_shoot_speed_level;
        m_last_shoot_speed = judge_package.m_last_shoot_speed;
        m_remain_time = judge_package.m_remain_time;
    }

    constexpr JudgePackage &operator=(const JudgePackage &judge_package) = default;

public:

    Buffer encode(JudgePackage judge_package) override
    {
        Buffer buffer;
        buffer.resize(8);
        buffer[0] = (judge_package.m_rune_status & 0x03) | ((judge_package.m_shoot_speed_level & 0x03) << 2);
        buffer[1] = judge_package.m_last_shoot_speed & 0xff;
        buffer[2] = judge_package.m_last_shoot_speed >> 8;
        buffer[3] = judge_package.m_remain_time & 0xff;
        buffer[4] = judge_package.m_remain_time >> 8;
        return buffer;
    }

    JudgePackage decode(Buffer buffer) override
    {
        JudgePackage judge_package;
        if(buffer.size() < 8) {
            std::cout << "JudgePackage recv buffer size less than 8" << std::endl;
            return judge_package;
        }
        judge_package.m_rune_status = buffer[0] & 0x03;
        judge_package.m_shoot_speed_level = (buffer[0] >> 2) & 0x03;
        judge_package.m_last_shoot_speed = ((int)buffer[1] | (int)buffer[2] << 8);
        judge_package.m_remain_time = ((int)buffer[3] | (int)buffer[4] << 8);

        return judge_package;
    }

    /**
     * @brief: 获取当前能量机关状态
     *
     * @retrun: ROBO_ENERGY ENERGY_IDLE 0x00不可激活, ENERGY_SMALL 0x01小能量, ENERGY_BIG 0x02大能量
     */
    ROBO_ENERGY getEnergyStatus()
    {
        return (ROBO_ENERGY)(m_rune_status);
    }

    /**
     * @brief: 接收弹速回传计算平均值用以弹道补偿的回传
     *
     * @return: float 弹速
     */
    float getShootSpeedValue()
    {
        if(m_clear_shoot_speed){
            m_last_shoot_speed = 0;
            m_clear_shoot_speed = false;
        }

        ROBO_SHOOT shoot_level = (ROBO_SHOOT)(m_shoot_speed_level);
        float f_shoot_speed = 0;
        f_shoot_speed = (int16_t)(m_last_shoot_speed) / 10.f;

        // 等级切换了 现在算出的弹速是上一等级的 弃用并清空队列 返回默认值
        // 为了读取实时弹速 Buffer会清空 设置判断条件SPEED_IDLE
        if (m_shoot_level != shoot_level && shoot_level != ROBO_SHOOT::SPEED_IDLE)
        {
            m_prev_shoot_speed.clear();
            m_shoot_level = shoot_level;
            m_shoot_speed_initialized = false;
            if (m_shoot_level == SPEED_HIGH)
            {
                return m_thirty_shoot_speed;
            }
            else if (m_shoot_level == SPEED_MID)
            {
                return m_eighteen_shoot_speed;
            }
            else
            {
                return m_fifteen_shoot_speed;
            }
        }

        // 回传不是0
        if (f_shoot_speed > 0)
        {
            m_shoot_speed_initialized = true;

            m_prev_shoot_speed.push_back(f_shoot_speed);
            if (m_prev_shoot_speed.size() == 11)
            {
                m_cur_shoot_speed = (10 * m_cur_shoot_speed - m_prev_shoot_speed.front() + m_prev_shoot_speed.back()) / 10.f;
                m_prev_shoot_speed.pop_front();
            }
            else
            {
                m_cur_shoot_speed = (m_cur_shoot_speed * (m_prev_shoot_speed.size() - 1) + f_shoot_speed) / m_prev_shoot_speed.size();
            }
            return m_cur_shoot_speed;
        }
        else // 等级切换后弹速回传仍是0
        {
            if (m_prev_shoot_speed.size() > 0)
                return m_cur_shoot_speed;
            if (m_shoot_level == SPEED_HIGH)
            {
                return m_thirty_shoot_speed;
            }
            else if (m_shoot_level == SPEED_MID)
            {
                return m_eighteen_shoot_speed;
            }
            else
            {
                return m_fifteen_shoot_speed;
            }
        }
    }

    /**
     * @brief: 返回当前等级的最高弹速
     *
     * @return: float
     */
    float getMaxShootSpeed()
    {
        if (m_shoot_level == SPEED_LOW)
        {
            return m_fifteen_shoot_speed;
        }
        else if (m_shoot_level == SPEED_MID)
        {
            return m_eighteen_shoot_speed;
        }
        else
        {
            return m_thirty_shoot_speed;
        }
    }
};

bool JudgePackage::m_shoot_speed_initialized = false;
ROBO_SHOOT JudgePackage::m_shoot_level = ROBO_SHOOT::SPEED_IDLE;
std::deque<float> JudgePackage::m_prev_shoot_speed;
float JudgePackage::m_cur_shoot_speed = 0;
float JudgePackage::m_fifteen_shoot_speed;
float JudgePackage::m_eighteen_shoot_speed;
float JudgePackage::m_thirty_shoot_speed; 
bool JudgePackage::m_clear_shoot_speed = false;

#endif // __JUDGE_PACKAGE_HPP__