#ifndef __PACKAGE_IDS_DEFINE_HPP__
#define __PACKAGE_IDS_DEFINE_HPP__

#include "impls/PackageID.hpp"

/**
 * @brief 包ID的定义文件，注意只在该文件中定义包ID，不要直接include该文件。
 * 对包ID的引用应通过 #include "impls/PackageID.hpp" 实现
 * 
 * @note 以下依次罗列要定义的Package ID名，要求命名采用全大写，中间用下划线隔开。
 * 由于以下全部以全局变量的格式定义，在使用中注意命名冲突
 * 以下Package ID在编译和初始化时没有实际对应的通信包id，需要在config.xml文件里编辑name关键字
 * ！！！注意xml里和这里命名一一对应，一个字母都不能差！！！
*/

ADD_PACKAGE(CAN_ID_CHASSIS)
ADD_PACKAGE(CAN_ID_GIMBAL)
ADD_PACKAGE(CAN_ID_GYRO)
ADD_PACKAGE(CAN_ID_SHOOT)
ADD_PACKAGE(SERIAL_ID_JUDGE)
ADD_PACKAGE(SERIAL_ID_SHOOT)
ADD_PACKAGE(TEST_TIME)


#endif // __PACKAGE_IDS_DEFINE_HPP__