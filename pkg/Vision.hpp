#ifndef __VISION_PACKAGE_HPP__
#define __VISION_PACKAGE_HPP__

#include "PackageInterface.hpp"

namespace transport{

#pragma pack(1)
struct PVision
{
    uint8_t current_mode : 3;       //视觉当前模式
    uint8_t current_exposure : 3;   //视觉当前曝光
    uint8_t auto_shoot : 1;         //是否使用自动击发
    uint8_t place_holder_1 : 2;     
    uint8_t armor_id : 8;           //识别到的装甲板id
    uint8_t recognized : 1;         //是否识别到
    uint8_t light_ok : 1;           //灯条有问题  
    uint8_t light_compare_ok : 1;   //灯条配对有问题
    uint8_t number_ok : 1;          //数字识别有问题
    uint8_t double_camera : 1;      //单双目有问题
    uint8_t error_item : 1;         //未识别到相机编号
    uint8_t armor_compare_ok : 1;   //装甲板双目配对有问题
    uint8_t select_ok : 1;          //筛选器筛除
    uint8_t trigger_enable : 8;     //是否开启硬触发
    uint8_t delay_time : 8;         //触发相机到发送IMU信息的时间延时
    uint8_t rate : 8;               //触发频率

    TRANSFORM_FUNC(PVision)
};
#pragma pack()

class VisionPackage : public PackageInterFace<VisionPackage>
{
private:
    PVision version_msg;
public:

    void encode(VisionPackage &version_package, Buffer& buffer) override
    {
        buffer.resize(sizeof(PVision));

        buffer << version_package.version_msg;

        return;
    }

    void decode(VisionPackage &version_package, Buffer& buffer) override
    {
        if(buffer.size() < 8) {
            LOGWARN("VisionPackage recv buffer size less than 8");
            return;
        }
        
        version_package.version_msg << buffer;

        return;
    }

    void SendVisionCurrentStatue(ROBO_STATE Mode, CAMERA_EXPOSURE exposure, bool auto_shoot_ok, int id,
                                                  bool ifrecognize, bool light_ok, bool light_compare_ok, bool number_ok,
                                                  int cameranum, int erroritem, bool armor_compare_ok, bool select_ok)
    {

        // std::cout << "auto_shoot:" << auto_shoot_ok << std::endl
        //           << "recognize:" << ifrecognize << std::endl
        //           << "light:" << light_ok << std::endl
        //           << "light_compare:" << light_compare_ok << std::endl
        //           << "numdetect:" << number_ok << std::endl
        //           << "doubleye:" << cameranum << std::endl
        //           << "errorcamera:" << erroritem << std::endl
        //           << "armor_compare:" << armor_compare_ok << std::endl
        //           << "select:" << select_ok << std::endl;

        version_msg.auto_shoot          = 0x01 & auto_shoot_ok;
        version_msg.recognized          = 0x01 & ifrecognize;
        version_msg.light_ok            = 0x01 & light_ok;
        version_msg.light_compare_ok    = 0x01 & light_compare_ok;
        version_msg.number_ok           = 0x01 & number_ok;
        version_msg.double_camera       = 0x01 & (cameranum >> 1);
        version_msg.error_item          = 0x01 & erroritem;
        version_msg.armor_compare_ok    = 0x01 & armor_compare_ok;
        version_msg.select_ok           = 0x01 & select_ok;
    }

    void CameraTrigger(bool v_trigger_enable, int v_delay_time, int v_rate)
    {
        LOGINFO("start trigger camera");

        version_msg.trigger_enable = v_trigger_enable;
        version_msg.delay_time = v_delay_time;
        version_msg.rate = v_rate;
    }
};

} // namespace transport

#endif // __VERSION_PACKAGE_HPP__