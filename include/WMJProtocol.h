#pragma once
#include <vector>
#include <time.h>

#include <queue>
#include <memory>

#include <sys/time.h>

typedef std::vector<uint8_t> Buffer;
typedef std::queue<Buffer> BufferQueue;
typedef std::pair<Buffer, int> BufferWithID;
typedef std::queue<BufferWithID> BufferWithIDQueue;
typedef std::pair<Buffer, timeval> BufferWithTime;
typedef std::queue<BufferWithTime> BufferWithTimeQueue;
typedef std::shared_ptr<BufferWithTimeQueue> BufferWithTimeQueuePtr;

template <typename T>
inline void clear(T *t)
{
    memset(t, 0, sizeof(T));
}

enum CAN_ID
{
    GIMBAL = 0x314,
    SHOOT = 0x321
};


// 接口信息表
struct PortStatus
{
    std::string port_name;  // 接口名
    int status;     // 可用状态 1:可用, 0:不可用, -1:该口已经迁移完成
    int workload;   // 接口工作负载
};

// 接口负载
class Workload
{
private:
    time_t m_last_sec;
    int m_last_sec_count;
    int m_cur_count;
    bool m_update;

public:
    void update()
    {
        timeval tv;
        gettimeofday(&tv, NULL);
        if(m_last_sec == tv.tv_sec)
            ++m_cur_count;
        else
        {
            m_update = true;
            m_last_sec = tv.tv_sec;
            m_last_sec_count = m_cur_count;
            m_cur_count = 0;
        }
    }

    int getWorkload()
    {
        m_update = false;
        return m_last_sec_count;
    }

    bool canUpload()
    {
        return m_update;
    }
};

#ifdef USE
#pragma pack(1)

/**
 * @brief 在结构体中，成员按顺序从低位向高位排列
 * 用‘：’定义结构体成员时，默认依次按位排列，从低位向高位排列
 * 直接使用默认变量类型时，会对齐字节，默认按4字节对齐，使用#pragma pack(1)指定后按1字节对齐
 * 成员全使用char或char[]时，默认按1字节对齐
 *
 */

struct GimbalPose
{
    float pitch;
    float yaw;
    float roll;
};

struct Client
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
    char key;
};

struct KeyBoard
{
    char A : 1;
    char B : 1;
    char C : 1;
    char D : 1;
    char E : 1;
    char F : 1;
    char G : 1;
    char H : 1;
    char I : 1;
    char J : 1;
};

struct ShooterControlMsg
{
    uint8_t shoot_mode;
    uint8_t shoot_num;
    uint16_t shoot_rub_speed;
    uint16_t shoot_boost_speed;
    uint8_t placehoder1;
    uint8_t placehoder2;
};

#pragma pack()

namespace wmj
{
    //这里用的是下相机的接口
    // 设备枚举
    enum DeviceID
    {
        NONE,
        Gimbal,
        Shooter,
        Chassis,
        MainControl,
        Gyro,
        Draw,
        Judge,
        ComSend,
        ComRecv,
        GyroAngle,
        GyroAngularVelocity,
        GyroAcce,
        Vision,
        State
    };

    // 机器人id
    enum ROBO_ID
    {
        ID_HERO = 0x00,
        ID_ENGINERR = 0x01,
        ID_INFANTRY3 = 0x02,
        ID_INFANTRY4 = 0x03,
        ID_INFANTRY5 = 0x04,
        ID_SENTRY = 0x05,
    };

    // 视觉指令
    enum ROBO_STATE
    {
        STATE_ARMOR = 0x00,
        STATE_TOP = 0x01,
        STATE_RUNE = 0x02,
        STATE_DARK = 0x03,
    };

    enum CAMERA_EXPOSURE
    {
        EXPOSURE_LOW = 0x00,
        EXPOSURE_MIDDLE = 0x01,
        EXPOSURE_HIGH = 0x02,
    };

    // 操作
    enum ROBO_OPT
    {
        OPT_NONE = 0x00,
        OPT_ADD = 0x01,
        OPT_FIX = 0x02,
        OPT_DELETE = 0x03,
    };

    // 颜色
    enum ROBO_COLOR
    {
        COLOR_MAIN = 0x00,
        COLOR_YELLOW = 0x01,
        COLOR_PINK = 0x02,
        COLOR_WHITE = 0x03,
    };

    // 形状
    enum ROBO_SHAPE
    {
        SHAPE_LINE = 0x00,
        SHAPE_RECT = 0x01,
        SHAPE_OVAL = 0x02,
        SHAPE_CHAR = 0x03,
    };

    // 图层
    enum ROBO_LAYER
    {
        LAYER_0 = 0x00,
        LAYER_1 = 0x01,
        LAYER_2 = 0x02,
        LAYER_3 = 0x03,
    };

    // 删除图形
    enum ROBO_DELETE
    {
        DELETE_NONE = 0x00,  // 空操作
        DELETE_DEL = 0x01,   // 删除
        DELETE_CLEAR = 0x02, // 清空
    };

    // 比赛状态
    enum ROBO_GAME
    {
        GAME_NOTSTART = 0x00, // 未开始
        GAME_PREPARE = 0x01,  // 准备阶段
        GAME_CHECK = 0x02,    // 自检
        GAME_5SEC = 0x03,     // 5s倒计时
        GAME_BATTLE = 0x04,   // 开始比赛
        GAME_END = 0x05,      // 比赛结束
    };

    // 能量机关状态
    enum ROBO_ENERGY
    {
        ENERGY_IDLE = 0x00,
        ENERGY_SMALL = 0x01,
        ENERGY_BIG = 0x02,
    };

    // 射击速度
    enum ROBO_SHOOT
    {
        SPEED_IDLE = 0x00,
        SPEED_LOW = 0x01,
        SPEED_MID = 0x02,
        SPEED_HIGH = 0x03,
    };

    // 机器人增益
    enum ROBO_GAIN
    {
        GAIN_HEAL = 0x00,   // 补血
        GAIN_CHILL = 0x01,  // 枪口冷血
        GAIN_SHIELD = 0x02, // 防御
        GAIN_ATTACK = 0x03, // 伤害
    };

    // RFID增益点
    enum ROBO_RFID
    {
        RFID_BASE = 0x00,     // 基地
        RFID_HILL = 0x01,     // 高地
        RFID_BUFF = 0x02,     // 能量机关
        RFID_SLOPE = 0x03,    // 飞坡
        RFID_OUTPOST = 0x04,  // 前哨战
        RFID_RESOURCE = 0x05, // 资源岛
        RFID_HEAL = 0x06,     // 补血
        RFID_ENGHEAL = 0x07,  // 工程补血
    };

    // 底盘功能
    enum CHA_FUNC
    {
        CHA_AUTO = 0x01,  // 自动步兵
        CHA_TOP = 0x02,   // 小陀螺
        CHA_POWER = 0x04, // 功率限制
        CHA_LOB = 0x08,   // 吊射
        CHA_SLOPE = 0x10, // 飞坡
    };

    /////////////////////////////主控信息//////////////////////////////
    struct MainCtlMsg
    {
        uint8_t mctl_base;
        uint8_t mctl_distance_low;
        uint8_t mctl_distance_high;
        uint8_t mctl_if_end_exposure;
        uint8_t placehoder1;
        uint8_t placehoder2;
        uint8_t placehoder3;
        uint8_t placehoder4;

        MainCtlMsg() : mctl_base{0},
                       mctl_distance_low{0},
                       mctl_distance_high{0},
                       mctl_if_end_exposure{0},
                       placehoder1{0},
                       placehoder2{0},
                       placehoder3{0},
                       placehoder4{0}
        {
        }
    };

    union MCMPack
    {
        MainCtlMsg msg;
        uint8_t data[sizeof(msg)];
        MCMPack() : msg{} {}
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

        bool up;    // W
        bool down;  // S
        bool left;  // A
        bool right; // D
    };

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

    /////////////////////////////裁判系统//////////////////////////////
    // 绘图
    struct DrawCtlMsg
    {
        uint8_t image_opt1; // High -> Low : color(2) layer(2) shape(2) opt(2)
        uint8_t image_opt2; // High -> Low : 0bit(2) reciever(3) delete_all(1)
        uint8_t start_x_low8;
        uint8_t start_x_high4_start_y_low4;
        uint8_t start_y_high8;
        uint8_t end_x_low8;
        uint8_t end_x_high4_end_y_low4;
        uint8_t end_y_high8;

        DrawCtlMsg() : image_opt1{0},
                       image_opt2{0},
                       start_x_low8{0},
                       start_x_high4_start_y_low4{0},
                       start_y_high8{0},
                       end_x_low8{0},
                       end_x_high4_end_y_low4{0},
                       end_y_high8{0}
        {
        }
    };

    union DCMPack
    {
        DrawCtlMsg msg;
        uint8_t data[sizeof(msg)];
        DCMPack() : msg() {}
    };

    // 接收信息
    struct JudgeCtlMsg
    {
        uint8_t infos1; // High -> Low : 增益(2) 虚拟护盾(1) 大能量(1) 小能量(1) 比赛阶段(3)
        uint8_t infos2; // High -> Low : 机器人增益RFID(3)
        uint8_t hp_low8;
        uint8_t hp_high4_base_hp_low4;
        uint8_t base_hp_high8;
        uint8_t outpost_hp;
        uint8_t placehoder1;
        uint8_t placehoder2;

        JudgeCtlMsg() : infos1{0},
                        infos2{0},
                        hp_low8{0},
                        hp_high4_base_hp_low4{0},
                        base_hp_high8{0},
                        outpost_hp{0},
                        placehoder1{0},
                        placehoder2{0}
        {
        }
    };

    union JCMPack
    {
        JudgeCtlMsg msg;
        uint8_t data[sizeof(msg)];
        JCMPack() : msg() {}
    };

    // 机器人通讯
    struct CommunicateCtlMsg
    {
        uint8_t info; // High -> Low : 增益(2) 虚拟护盾(1) 大能量(1) 小能量(1) 比赛阶段(3)
        uint8_t test; // High -> Low : 机器人增益RFID(3)
        uint8_t placehoder1;
        uint8_t placehoder2;
        uint8_t placehoder3;
        uint8_t placehoder4;
        uint8_t placehoder5;
        uint8_t placehoder6;

        CommunicateCtlMsg() : info{0},
                              test{0},
                              placehoder1(0),
                              placehoder2(0),
                              placehoder3(0),
                              placehoder4(0),
                              placehoder5(0),
                              placehoder6(0)
        {
        }
    };

    union CCMPack
    {
        CommunicateCtlMsg msg;
        uint8_t data[sizeof(msg)];
        CCMPack() : msg() {}
    };

    ////////////////////////////发射信息//////////////////////////////
    struct ShooterControlMsg
    {
        uint8_t shot_mode;
        uint8_t shot_num;
        uint8_t shot_rub_speed_low;
        uint8_t shot_rub_speed_high;
        uint8_t shot_boost_speed_low;
        uint8_t shot_boost_speed_high;
        uint8_t placehoder1;
        uint8_t placehoder2;

        ShooterControlMsg() : shot_mode{0},
                              shot_num{0},
                              shot_rub_speed_low{0},
                              shot_rub_speed_high{0},
                              shot_boost_speed_low{0},
                              shot_boost_speed_high{0},
                              placehoder1{0},
                              placehoder2{0}
        {
        }
    };

    union SCMPack
    {
        ShooterControlMsg msg;
        uint8_t data[sizeof(msg)];
        SCMPack() : msg{} {}
    };

    /////////////////////////////云台信息///////////////////////////////
    //////////////////////下云台信息//////////////////////////
    struct GimbalControlMsg
    {
        uint8_t gm_mode;
        uint8_t gm_yaw_velocity;
        uint8_t gm_both_velocity;
        uint8_t gm_pitch_velocity;
        uint8_t gm_yaw_angle_low;
        uint8_t gm_yaw_angle_high;
        uint8_t gm_pitch_angle_low;
        uint8_t gm_pitch_angle_high;

        GimbalControlMsg() : gm_mode{1},
                             gm_yaw_velocity{0},
                             gm_both_velocity{0},
                             gm_pitch_velocity{0},
                             gm_yaw_angle_low{0},
                             gm_yaw_angle_high{0},
                             gm_pitch_angle_low{0},
                             gm_pitch_angle_high{0}
        {
        }
    };

    union GCMPack
    {
        GimbalControlMsg msg;
        uint8_t data[sizeof(msg)];
        GCMPack() : msg{} {}
    };

    /////////////////////////视觉通讯信息///////////////////////////////
    struct VisionSentMsg
    {
        uint8_t vision_Mode;
        uint8_t vision_Armorid;
        uint8_t vision_Recognize;
        uint8_t vision_CameraTriggerEnable;
        uint16_t vision_CameraRate;
        uint16_t vision_CameraDlay;
        VisionSentMsg() : vision_Mode{0},
                          vision_Armorid{0},
                          vision_Recognize{0},
                          vision_CameraTriggerEnable{0},
                          vision_CameraRate{0},
                          vision_CameraDlay{0}
        {
        }
    };

    union VSMPack
    {
        VisionSentMsg msg;
        uint8_t data[sizeof(msg)];
        VSMPack() : msg{} {}
    };

    /////////////////////////////底盘陀螺仪信息///////////////////////////////
    struct GyroAngleMsg
    {
        uint8_t timestamp_low;
        uint8_t timestamp_high;
        uint8_t yaw_angle_low;
        uint8_t yaw_angle_high;
        uint8_t pitch_angle_low;
        uint8_t pitch_angle_high;
        uint8_t roll_angle_low;
        uint8_t roll_angle_high;
    };

    union GyroAnglePack
    {
        GyroAngleMsg msg;
        uint8_t data[sizeof(msg)];
        GyroAnglePack() : msg{} {}
    };

    struct GyroAngularVelocityMsg
    {
        uint8_t timestamp_low;
        uint8_t timestamp_high;
        uint8_t yaw_velocity_low;
        uint8_t yaw_velocity_high;
        uint8_t pitch_velocity_low;
        uint8_t pitch_velocity_high;
        uint8_t roll_velocity_low;
        uint8_t roll_velocity_high;
    };

    union GyroAngularVelocityPack
    {
        GyroAngularVelocityMsg msg;
        uint8_t data[sizeof(msg)];
        GyroAngularVelocityPack() : msg{} {}
    };

    struct GyroAcceMsg
    {
        uint8_t timestamp_low;
        uint8_t timestamp_high;
        uint8_t x_acc_low;
        uint8_t x_acc_high;
        uint8_t y_acc_low;
        uint8_t y_acc_high;
        uint8_t z_acc_low;
        uint8_t z_acc_high;
    };

    union GyroAccePack
    {
        GyroAcceMsg msg;
        uint8_t data[sizeof(msg)];
        GyroAccePack() : msg{} {};
    };

    /////////////////////////////底盘信息///////////////////////////////
    struct ChassisControlMsg
    {
        uint8_t cha_info1;
        uint8_t cha_x_speed_high;
        uint8_t cha_x_speed_low;
        uint8_t cha_y_speed_high;
        uint8_t cha_y_speed_low;
        uint8_t cha_placehoder1;
        uint8_t cha_placehoder2;
        uint8_t cha_placehoder3;

        ChassisControlMsg() : cha_info1{0},
                              cha_x_speed_high{0},
                              cha_x_speed_low{0},
                              cha_y_speed_high{0},
                              cha_y_speed_low{0},
                              cha_placehoder1{0},
                              cha_placehoder2{0},
                              cha_placehoder3{0}
        {
        }
    };

    union CHMPack
    {
        ChassisControlMsg msg;
        uint8_t data[sizeof(msg)];
        CHMPack() : msg{} {}
    };
    //////////////////////////////状态信息回传///////////////////////////////

    struct KeyMsg
    {
        uint8_t W : 1;
        uint8_t S : 1;
        uint8_t A : 1;
        uint8_t D : 1;
        uint8_t SHIFT : 1;
        uint8_t CTRL : 1;
        uint8_t Q : 1;
        uint8_t E : 1;
        uint8_t R : 1;
        uint8_t F : 1;
        uint8_t G : 1;
        uint8_t Z : 1;
        uint8_t X : 1;
        uint8_t C : 1;
        uint8_t V : 1;
        uint8_t B : 1;

        // GameStateMsg() :
        // W(0), S(0), A(0), D(0),
        // Q(0), E(0), R(0), F(0),
        // G(0), Z(0), X(0), C(0),
        // V(0), B(0), SHIFT(0), CTRL(0)
        // {}
    };

    union KeyMsgPack
    {
        KeyMsg msg;
        uint8_t data[sizeof(msg)];
        KeyMsgPack()
        {
            memset(data, 0, sizeof(msg));
        }
    };

    struct GameStateMsg
    {
        uint8_t game_start;     // 0~7 比赛开始结束：1 己方前哨站状态：1 RFID状态（哨兵巡逻区）：1 RFID状态（高地增益）：1
        uint8_t outpost_state;  //前哨站
        uint8_t RFID_patrol;    //巡逻区
        uint8_t RFID_elevated;  //高地RFID，ps：官方高地英语 Elevated Ground
        uint8_t armor_id;       //装甲id ： 1～4
        uint8_t harm_type;      //伤害类型
        uint16_t sentry_blood;  //哨兵血量
        uint8_t bullet_remain;  //剩余子弹发射数
        uint16_t outpost_blood; // 前哨站血量
        uint16_t hityaw;        //装甲受击yaw方向
        uint16_t remain_time;   //比赛剩余时间
        KeyMsg key;             //按鍵信息
        uint8_t own_alive;      //己方机器人存活
        uint8_t enemy_alive;    //敌方机器人存活

        GameStateMsg() : game_start{0},
                         outpost_state{0},
                         RFID_patrol{0},
                         RFID_elevated{0},
                         armor_id{0},
                         harm_type{0},
                         sentry_blood{0},
                         bullet_remain{0},
                         hityaw{0},
                         outpost_blood{0},
                         remain_time{0},
                         own_alive{0},
                         enemy_alive{0}
        {
        }
    };

}
/* struct ChassisRecvMsg
 {
     uint8_t cha_info1;
     uint8_t cha_x_speed_high;
     uint8_t cha_x_speed_low;
     uint8_t cha_y_speed_high;
     uint8_t cha_y_speed_low;
     uint8_t cha_placehoder1;
     uint8_t cha_placehoder2;
     uint8_t cha_placehoder3;

     ChassisRecvMsg():
         cha_info1{0},
         cha_x_speed_high{0},
         cha_x_speed_low{0},
         cha_y_speed_high{0},
         cha_y_speed_low{0},
         cha_placehoder1{0},
         cha_placehoder2{0},
         cha_placehoder3{0}
     {}
 };
 union CHRPack
 {
     ChassisRecvMsg msg;
     uint8_t data[sizeof(msg)];
     CHRPack(): msg{} {}
 };
 */
#endif // USE