#ifndef __PROTOCAL_HPP__
#define __PROTOCAL_HPP__

// 端口类型
enum PORT_TYPE{
    NONE = 0x00,
    CAN = 0x01,
    SERIAL = 0x02
};

// 机器人id
enum ROBO_ID
{
    ID_HERO      = 0x00,
    ID_ENGINERR  = 0x01,
    ID_INFANTRY3 = 0x02,
    ID_INFANTRY4 = 0x03,
    ID_INFANTRY5 = 0x04,
    ID_SENTRY    = 0x05,
};

// 视觉指令
enum ROBO_STATE
{
    STATE_ARMOR   	  = 0x00,
    STATE_RUNE    	  = 0x01,
    STATE_DARK    	  = 0x02,
    STATE_NONE        = 0x03,
};

enum CAMERA_EXPOSURE
{
    EXPOSURE_LOW    = 0x00,
    EXPOSURE_MIDDLE = 0x01,
    EXPOSURE_HIGH   = 0x02,
};

// 操作
enum ROBO_OPT
{
    OPT_NONE   = 0x00,
    OPT_ADD    = 0x01,
    OPT_FIX    = 0x02,
    OPT_DELETE = 0x03,
};

// 颜色
enum ROBO_COLOR
{
    COLOR_MAIN   = 0x00,
    COLOR_YELLOW = 0x01,
    COLOR_PINK   = 0x02,
    COLOR_WHITE  = 0x03,
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
    DELETE_NONE  = 0x00, // 空操作
    DELETE_DEL   = 0x01, // 删除
    DELETE_CLEAR = 0x02, // 清空
};

// 比赛状态
enum ROBO_GAME
{
    GAME_NOTSTART = 0x00, // 未开始
    GAME_PREPARE  = 0x01, // 准备阶段
    GAME_CHECK    = 0x02, // 自检
    GAME_5SEC     = 0x03, // 5s倒计时
    GAME_BATTLE   = 0x04, // 开始比赛
    GAME_END      = 0x05, // 比赛结束
};

// 能量机关状态
enum ROBO_ENERGY
{
    ENERGY_IDLE  = 0x00,
    ENERGY_SMALL = 0x01,
    ENERGY_BIG   = 0x02,
};

// 射击速度
enum ROBO_SHOOT
{
    SPEED_IDLE = 0x00,
    SPEED_LOW  = 0x01,
    SPEED_MID  = 0x02,
    SPEED_HIGH = 0x03,
};

// 机器人增益
enum ROBO_GAIN
{
    GAIN_HEAL   = 0x00, // 补血
    GAIN_CHILL  = 0x01, // 枪口冷血
    GAIN_SHIELD = 0x02, // 防御
    GAIN_ATTACK = 0x03, // 伤害
};

// RFID增益点
enum ROBO_RFID
{
    RFID_BASE     = 0x00, // 基地
    RFID_HILL     = 0x01, // 高地
    RFID_BUFF     = 0x02, // 能量机关
    RFID_SLOPE    = 0x03, // 飞坡
    RFID_OUTPOST  = 0x04, // 前哨战
    RFID_RESOURCE = 0x05, // 资源岛
    RFID_HEAL     = 0x06, // 补血
    RFID_ENGHEAL  = 0x07, // 工程补血
};

// 底盘功能
enum CHA_FUNC
{
    CHA_AUTO  = 0x01, // 自动步兵
    CHA_TOP   = 0x02, // 小陀螺
    CHA_POWER = 0x04, // 功率限制
    CHA_LOB   = 0x08, // 吊射
    CHA_SLOPE = 0x10, // 飞坡
};

enum DEBUG_FLAG
{
    DEBUG_PRINT_ID_IF_RECEIVED = 0x01,
    DEBUG_PRINT_BUFFER = 0x02,
    DEBUG_PRINT_TARGET = 0x04,
};

#endif // __PROTOCAL_HPP__