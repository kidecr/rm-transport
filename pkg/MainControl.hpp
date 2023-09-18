#ifndef __MAIN_CONTROL_PACKAGE_HPP__
#define __MAIN_CONTROL_PACKAGE_HPP__

#include "PackageInterface.hpp"

namespace transport{
// 对应主控信息(DBUS)

#pragma pack(1)
struct PMainControl
{
    // data[0]
    uint8_t dbus_online : 1;        // DBUS在线
    uint8_t place_holder_1 : 7;
    // data[1]
    uint8_t S1 : 2;                 // 数据1~3分别代表左拨杆高低中
    uint8_t S2 : 2;                 // 数据1~3分别代表右拨杆高低中
    uint8_t mouse_left_click : 1;   // 鼠标左键 0为松开，1为按下
    uint8_t mouse_right_click : 1;  // 鼠标右键 0为松开，1为按下
    uint8_t place_holder_2 : 2;
    // data[2,3], data[4,5]
    uint16_t mouse_X_speed : 16;    // 鼠标X轴速度
    uint16_t mouse_Y_speed : 16;    // 鼠标Y轴速度
    // data[6]
    uint8_t W : 1;
    uint8_t S : 1;
    uint8_t A : 1;
    uint8_t D : 1;
    uint8_t SHIFT : 1;
    uint8_t CTRL : 1;
    uint8_t Q : 1;
    uint8_t E : 1;
    // data[7]
    uint8_t R : 1;
    uint8_t F : 1;
    uint8_t G : 1;
    uint8_t Z : 1;
    uint8_t X : 1;
    uint8_t C : 1;
    uint8_t V : 1;
    uint8_t B : 1;

    TRANSFORM_FUNC(PMainControl)
};
#pragma pack()

struct KeyboardClick
{
    // 按键单击判断bool值,命名为该按键单击
    bool click_w = false;
    bool click_s = false;
    bool click_a = false;
    bool click_d = false;
    bool click_q = false;
    bool click_e = false;
    bool click_z = false;
    bool click_x = false;
    bool click_g = false;
};

struct KeyboardInfo
{
    // 多按键bool综合值,若想得到相应的按键状态,请使用bool运算
    bool aim_armor;         // 0x29为自瞄开始条件,s1为1,s2为2,鼠标右键active
    bool enable_shoot;      //击发
    bool auto_shoot_switch; // 自动击发开关,更改自动击发状态
    bool aim_last;          // 逆向切换视觉状态
    bool aim_next;          // 正向切换视觉状态
    bool aim_rune_em;       // 无UI紧急打符模式，true直接进入打符模式
    bool aim_reset;         // 重置视觉状态为默认状态(armor)
    bool change_exposure;   // 相机曝光等级切换
    bool use_number_detect;

    bool up;                // W
    bool down;              // S
    bool left;              // A
    bool right;             // D
};

class MainControlPackage : public PackageInterFace<MainControlPackage>
{
public:
    PMainControl m_main_control_msg;

    static ROBO_STATE m_cur_robo_state;
    static ROBO_STATE m_last_state;
    static CAMERA_EXPOSURE m_armor_camera_exposure;
    static CAMERA_EXPOSURE m_dark_camera_exposure;
    static bool m_aim_armor;
    static bool m_enable_shoot;
    static bool m_auto_shoot;
    static bool m_use_number_detect;
    static KeyboardClick m_click;
    static wmj::GimbalPose m_adjust_pose;

public:
    MainControlPackage()
    {
        clear(&m_main_control_msg);
    }

    MainControlPackage(const MainControlPackage &main_control_package)
    {
        this->m_main_control_msg = main_control_package.m_main_control_msg;
    }

    constexpr MainControlPackage &operator=(const MainControlPackage &main_control_package) = default;

/***********************解码*******************************/
    MainControlPackage decode(Buffer buffer) override
    {
        MainControlPackage main_control_package;
        if(buffer.size() < sizeof(PMainControl)) {
            LOGWARN("MainControlPackage recv buffer size less than 8");
            return main_control_package;
        }
        
        PMainControl main_control_msg;
        main_control_msg << buffer;
        main_control_package.m_main_control_msg = main_control_msg;
        return main_control_package;
    }

    /**
     * @brief 获取当前按键信息
     *
     * @return KeyboardInfo
     */
    KeyboardInfo GetKeyboardInfo()
    {

        // bool aim_a, aim_r0, aim_r1, Rins, Rdes;
        KeyboardInfo Info;
        /*
            视觉模式顺序
            ARMOR   普通自瞄
            TOP     反陀螺
            RUNE    打符
            DARK    打哨兵
        */

        // 自瞄模式
        // data1 0x05 左上右上
        // data1 0x09 左上右下
        // data1 0x20 右键
        // data1 0x30 左右键
        // 视觉模式正向切换 E
        // data6 0x80 E
        // 视觉模式逆向切换 Q
        // data6 0x40 Q
        // 应急复位按键，视觉模式重置为armor G
        // data7 0x04 G
        // 应急打符按键，按住为打符模式
        // data7 0x02 F
        // 正向曝光等级切换 Z
        // data7 0x08 Z
        // 自动击发开关 X
        // data7 0x10 X

        // 更改曝光值 Z 右键不能按下
        // data7 0x08 Z
        bool flag_z = !(m_main_control_msg.mouse_right_click) && (m_main_control_msg.Z);
        checkCLickAndSetStatus(flag_z, m_click.click_z, Info.change_exposure);

        // 自动击发开关 X
        // data7 0x10 X
        bool flag_x = m_main_control_msg.X;
        checkCLickAndSetStatus(flag_x, m_click.click_x, Info.auto_shoot_switch);

        // 视觉模式正向切换 E
        // Q、G、右键不能按下
        // data6 0x80 E
        bool flag_e = (m_main_control_msg.E) &&
                        !(m_main_control_msg.Q) &&
                        !(m_main_control_msg.G) &&
                        !(m_main_control_msg.mouse_right_click);
        checkCLickAndSetStatus(flag_e, m_click.click_e, Info.aim_next);

        // 视觉模式逆向切换 Q
        // E、G、右键不能按下
        // data6 0x40 Q
        bool flag_q = (m_main_control_msg.Q) &&
                        !(m_main_control_msg.E) &&
                        !(m_main_control_msg.G) &&
                        !(m_main_control_msg.mouse_right_click);
        checkCLickAndSetStatus(flag_q, m_click.click_q, Info.aim_last);

        // 视觉模式重置为默认 G
        // E、Q、右键不能按下
        // data7 0x04 G
        bool flag_g = (m_main_control_msg.G) &&
                        !(m_main_control_msg.E) &&
                        !(m_main_control_msg.Q) &&
                        !(m_main_control_msg.mouse_right_click);
        checkCLickAndSetStatus(flag_g, m_click.click_g, Info.aim_reset);

        // 键鼠模式按右键
        // data1 0x20 右键
        if (m_main_control_msg.mouse_right_click)
        {
            // 应急打符按键 按住F
            // data7 0x02 F
            if (m_main_control_msg.F)
            {
                Info.aim_rune_em = true;
            }
            else
            {
                Info.aim_rune_em = false;
            }

            // 击发
            // data1 0x10 左键
            if (m_main_control_msg.mouse_left_click)
            {
                Info.enable_shoot = true;
            }
            else
            {
                Info.enable_shoot = false;
            }
            Info.use_number_detect = true;
            Info.aim_armor = true;
        }
        else
        {
            Info.aim_armor = false;
            Info.enable_shoot = false;
            Info.use_number_detect = true;
        }

        // 微调位姿 w s a d
        bool flag_w = m_main_control_msg.W;
        checkCLickAndSetStatus(flag_w, m_click. click_w, Info.up);
        bool flag_s = m_main_control_msg.S;
        checkCLickAndSetStatus(flag_s, m_click. click_w, Info.down);
        bool flag_a = m_main_control_msg.A;
        checkCLickAndSetStatus(flag_a, m_click. click_w, Info.left);
        bool flag_d = m_main_control_msg.D;
        checkCLickAndSetStatus(flag_d, m_click. click_w, Info.right);

        // std::cout << "[info]"<< ((uint8_t)data_from_read[1] & 0x10) << " "
        //                      << ((uint8_t)data_from_read[1] & 0x20) << " "
        //                      << ((uint8_t)data_from_read[7] & 0x02) << " "
        //                      << ((uint8_t)data_from_read[1]) << " "
        //                      << ((uint8_t)data_from_read[7]) << std::endl
        //          << "[info]aim_armor:\t" << Info.aim_armor << std::endl
        //          << "[info]enable_shoot:\t" << Info.enable_shoot << std::endl
        //          << "[info]aim_last:\t" << Info.aim_last << std::endl
        //          << "[info]aim_next\t" << Info.aim_next << std::endl
        //          << "[info]change_exposure:\t" << Info.change_exposure << std::endl
        //          << "[info]use_number_detect:\t" << Info.use_number_detect << std::endl
        return Info;
    }

    /**
     *@brief: 获取发送的命令
     *
     *@return: ROBOSTATE 发送的命令
     */
    ROBO_STATE GetRobotStatus()
    {
        KeyboardInfo robotStatus;
        robotStatus = this->GetKeyboardInfo();
        bool rune_em = false;

        if (robotStatus.auto_shoot_switch) // 自动击发
            m_auto_shoot = !m_auto_shoot;  // 切换自动击发状态

        if (robotStatus.change_exposure) // 更改曝光值 打符模式下无效
        {
            if (m_cur_robo_state == STATE_DARK)
            {
                if (m_dark_camera_exposure == EXPOSURE_HIGH)
                    this->m_dark_camera_exposure = EXPOSURE_LOW;
                else
                    this->m_dark_camera_exposure = CAMERA_EXPOSURE(m_dark_camera_exposure + 1);
            }
            else if (m_cur_robo_state == STATE_ARMOR)
            {
                if (m_armor_camera_exposure == EXPOSURE_HIGH)
                    this->m_armor_camera_exposure = EXPOSURE_LOW;
                else
                    this->m_armor_camera_exposure = CAMERA_EXPOSURE(m_armor_camera_exposure + 1);
            }
        }

        if (robotStatus.aim_armor) // 启用自瞄
        {
            // 应急状态打符模式
            if (robotStatus.aim_rune_em)
                rune_em = true;

            if (robotStatus.enable_shoot) // 击发
            {
                this->m_enable_shoot = true;
                LOGINFO("ARMOR SHOOTING");
            }
            else
            {
                this->m_enable_shoot = false;
            }
            this->m_aim_armor = true;
        }
        else
        {
            this->m_aim_armor = false;
        }

        if (robotStatus.aim_next)
        {
            if (m_cur_robo_state == STATE_DARK)
                this->m_cur_robo_state = STATE_ARMOR;
            else
                this->m_cur_robo_state = ROBO_STATE(m_cur_robo_state + 1);
        }

        if (robotStatus.aim_last)
        {
            if (m_cur_robo_state == STATE_ARMOR)
                this->m_cur_robo_state = STATE_DARK;
            else
                this->m_cur_robo_state = ROBO_STATE(m_cur_robo_state - 1);
        }

        if (robotStatus.aim_reset)
        {
            this->m_cur_robo_state = STATE_ARMOR;
            this->m_armor_camera_exposure = EXPOSURE_MIDDLE;
            this->m_dark_camera_exposure = EXPOSURE_HIGH;
            this->m_auto_shoot = true;
            this->m_adjust_pose = wmj::GimbalPose(0, 0);
        }

        m_last_state = m_cur_robo_state;
        if (rune_em)
        {
            LOGINFO("[ROBO_STATE]\t%d", (int)ROBO_STATE::STATE_RUNE);

            if (robotStatus.up)
                this->m_adjust_pose.pitch -= 0.002;
            if (robotStatus.down)
                this->m_adjust_pose.pitch += 0.002;
            if (robotStatus.left)
                this->m_adjust_pose.yaw += 0.002;
            if (robotStatus.right)
                this->m_adjust_pose.yaw -= 0.002;

            LOGINFO("adjust_pose: [pitch: %f, yaw: %f]", m_adjust_pose.pitch, m_adjust_pose.yaw);

            return ROBO_STATE::STATE_RUNE;
        }
        else
        {
            LOGINFO("[ROBO_STATE]\t%d", (int)m_cur_robo_state);
            return m_cur_robo_state;
        }
    }

    /**
     * @brief 根据按键点击逻辑切换状态量。由于该逻辑复用过多，所以单独写成一个函数
     */
    void checkCLickAndSetStatus(bool flag, bool &click, bool &status)
    {
        if (flag)
        {
            // 如果按键当前按下且点击标志位为否，置真
            if (!click)
                click = true;
            status = false;
        }
        else
        {
            // 如果按键当前松开且点击标志位为真，即认为之前按键是按下状态，认为完成了一次点击
            // 将状态量置真，点击标志位复原
            if (click)
            {
                click = false;
                status = true;
            }
            else
            {
                status = false;
            }
        }
    }

    /**
     *@brief: 是否拉高射速
     *
     *@return: bool 是否拉高
     */
    bool HighShootSpeed()
    {
        if (m_main_control_msg.Z)
            return true;
        return false;
    }

/************************编码*******************************/
    Buffer encode(MainControlPackage main_control_package) override
    {
        Buffer buffer;
        buffer.resize(sizeof(PMainControl), 0);
        PMainControl main_control_msg;
        main_control_msg = main_control_package.m_main_control_msg;
        buffer << main_control_msg;
        return buffer;
    }
/***********************输出******************************/
    std::string toString() override
    {
        std::stringstream sstream;
        sstream << typeid(*this).name() << std::endl;

        sstream << "dbus_online: "      << m_main_control_msg.dbus_online           << std::endl; 

        sstream << "S1"                 << m_main_control_msg.S1                    << std::endl; 
        sstream << "S2"                 << m_main_control_msg.S2                    << std::endl; 
        sstream << "mouse_left_click"   << m_main_control_msg.mouse_left_click      << std::endl;
        sstream << "mouse_right_click"  << m_main_control_msg.mouse_right_click     << std::endl;
        
        sstream << "mouse_X_speed"      << m_main_control_msg.mouse_X_speed         << std::endl;
        sstream << "mouse_Y_speed"      << m_main_control_msg.mouse_Y_speed         << std::endl;    
        
        sstream << "W"                  << m_main_control_msg.W                     << std::endl;
        sstream << "S"                  << m_main_control_msg.S                     << std::endl;
        sstream << "A"                  << m_main_control_msg.A                     << std::endl;
        sstream << "D"                  << m_main_control_msg.D                     << std::endl;
        sstream << "SHIFT"              << m_main_control_msg.SHIFT                 << std::endl;
        sstream << "CTRL"               << m_main_control_msg.CTRL                  << std::endl;
        sstream << "Q"                  << m_main_control_msg.Q                     << std::endl;
        sstream << "E"                  << m_main_control_msg.E                     << std::endl;
        
        sstream << "R"                  << m_main_control_msg.R                     << std::endl;
        sstream << "F"                  << m_main_control_msg.F                     << std::endl;
        sstream << "G"                  << m_main_control_msg.G                     << std::endl;
        sstream << "Z"                  << m_main_control_msg.Z                     << std::endl;
        sstream << "X"                  << m_main_control_msg.X                     << std::endl;
        sstream << "C"                  << m_main_control_msg.C                     << std::endl;
        sstream << "V"                  << m_main_control_msg.V                     << std::endl;
        sstream << "B"                  << m_main_control_msg.B                     << std::endl;
        return sstream.str();
    }
};

KeyboardClick MainControlPackage::m_click;
ROBO_STATE MainControlPackage::m_cur_robo_state = STATE_ARMOR;
ROBO_STATE MainControlPackage::m_last_state;
CAMERA_EXPOSURE MainControlPackage::m_armor_camera_exposure = EXPOSURE_MIDDLE;
CAMERA_EXPOSURE MainControlPackage::m_dark_camera_exposure = EXPOSURE_HIGH;
bool MainControlPackage::m_aim_armor = false;
bool MainControlPackage::m_enable_shoot = false;
bool MainControlPackage::m_auto_shoot = true;
bool MainControlPackage::m_use_number_detect = true;
wmj::GimbalPose MainControlPackage::m_adjust_pose;

} // namespace transport

#endif // __MAIN_CONTROL_PACKAGE_HPP__