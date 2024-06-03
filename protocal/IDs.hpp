#ifndef __DEFINE_PACKAGE_HPP__
#define __DEFINE_PACKAGE_HPP__

#include <memory>
#include <map>
#include <string>
#include <cstdint>
#include "protocal/GlobalParam.hpp"

typedef uint64_t ID; // 定义ID类型

/**
 * @brief ID作为统一各种类型的包的标识，一定是端口ID，组ID和包ID共同组成的。
 *        结构为： [ reserve : 32 | group id : 8 | port id : 8 |package id : 16 ] 
*/
enum CAN_ID
{
    CHASSIS = 0x301,
    GIMBAL = 0x312,
    GYRO = 0x314,
    GIMBAL_GLOBAL = 0x316,
    SHOOT = 0x321,
    MAIN_CONTROL = 0x334,
    JUDGE = 0x344,
    TIME = 0x345,
};

enum SERIAL_ID
{
    BLOOD = 0x0003,
    TEST = 0x0233
};

class PackageID
{
public:
    struct PackageNameIdMapper
    {
        std::map<std::string, PackageID*> package_name_id_mapper;
    };
public:
    ID id;

    // PackageID(): id(0){};
    PackageID() = delete;
    PackageID(ID _id): id(_id){}
    PackageID(std::string package_id_name);

    operator ID() const;
    ID operator =(ID _id);

    static bool setPacakgeID(std::string package_name, ID value);
};

#define ADD_PACKAGE(PACKAGE_NAME) \
PackageID PACKAGE_NAME(#PACKAGE_NAME);

#define SET_PACKAGE(PACKAGE_NAME, VALUE) \
PackageID::setPacakgeID(PACKAGE_NAME, VALUE)

#define GET_PACKAGE(PACKAGE_NAME) \
GET_PARAM(PackageID::PackageNameIdMapper)->package_name_id_mapper[PACKAGE_NAME]->id;


/**
 * 以下依次罗列要定义的Package ID名，要求命名采用全大写，中间用下划线隔开。
 * 由于以下全部以全局变量的格式定义，在使用中注意命名冲突
 * 以下Package ID在编译和初始化时没有实际对应的通信包id，需要在config.xml文件里编辑name关键字
 * ！！！注意xml里和这里命名一一对应，一个字母都不能差！！！
*/

ADD_PACKAGE(CAN_ID_CHASSIS)
ADD_PACKAGE(CAN_ID_GIMBAL)
ADD_PACKAGE(SERIAL_ID_JUDGE)


// 用来约束IDType可以使用的类型
template<typename T>
concept IDType = std::is_same<T, CAN_ID>::value || 
                 std::is_same<T, SERIAL_ID>::value ||
                 std::is_same<T, ID>::value ||
                 std::is_same<T, PackageID>::value;

#endif // __DEFINE_PACKAGE_HPP__