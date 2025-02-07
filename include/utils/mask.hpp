#ifndef __MASK_HPP__
#define __MASK_HPP__

#include "protocal/Protocal.hpp"
#include "impls/PackageID.hpp"

namespace transport{


/**
 * @brief ID作为统一各种类型的包的标识，一定是端口ID，组ID和包ID共同组成的。
 *        结构为： [ reserve : 24 | group id : 8 | port id : 8 | device type : 8 | package id : 16 ] 
 * 其中 group id 按照配置文件中的先后顺序依次从0开始排列
 *      port id 按照配置文件中的先后顺序依次从0开始排列
 *      device type 分为can和serial两种，can对应0x01, serial对应0x02
 *      package id 由用户定义
*/


/**
 * @brief 提取PackageID中的id
 * 
 * @param package_id 
 * @return ID 
 */
inline ID mask(PackageID package_id)
{
    return static_cast<ID>(package_id.id);
}

/**
 * @brief 提供id转ID的类型转换，方便使用模板
 * 
 * @param id 
 * @return ID 
 */
inline ID mask(int id)
{
    return static_cast<ID>(id);
}


/**
 * @brief 提供包id转ID的类型转换
 * 
 * @param type 端口类型
 * @param id 包id
 * @param group_id port所在group id
 * @param port_id port所属id
 * @return ID 
 */
inline ID mask(PORT_TYPE type, ID _id, uint32_t group_id, uint32_t port_id)
{
    ID id = 0;
    uint32_t device_id = 0;
    if (type == PORT_TYPE::SERIAL)
        device_id = 0x02;
    if (type == PORT_TYPE::CAN)
        device_id = 0x01;
    id = group_id;
    id = id << 8;
    id = id | port_id;
    id = id << 8;
    id = id | device_id;
    id = id << 16;
    id = id | _id;
    return id;
}

// /**
//  * @brief 提供ID转ID的接口，方便使用模板
//  * 
//  */
// inline ID mask(ID id)
// {
//     return id;
// }

/**
 * @brief 去掉设备id，显示原id
 * 
 * @param id 
 * @return int 
 */
inline int unmask(ID id)
{
    int masked_id = static_cast<int>(id);
    return masked_id & 0x00ffff;
}

/**********************************************************
 * 配置文件中的id结构
 * 定义ID结构：0000 0000 0000 0000 0000 0000 0000 0000
 *          |         |         |                   |
 *          queue size   debug             id
 *
 */

/**
 * @brief 从配置文件的flag中获取debug flag
 * 
 * @param flag config.yaml中id除后4位之外的位
 * @return int 
 */
inline int get_debug_flag(int flag)
{
    return flag & 0xff;
}

/**
 * @brief 从配置文件的flag中获取queue size
 * 
 * @param flag config.yaml中id除后4位之外的位
 * @return int 
 */
inline int get_queue_size(int flag)
{
    auto queue_size = (flag >> 8) & 0xff;
    return queue_size > 1 ? queue_size : 1;
}

/**
 * @brief 从配置文件的id定义中获取实际id
 * 
 * @param id_info 配置文件的id定义
 * @return int 
 */
inline int get_package_id(int id_info)
{
    return id_info & 0xffff;
}

/**
 * @brief 从配置文件的id定义中获取配置信息flag
 * 
 * @param id_info 配置文件的id定义
 * @return int 
 */
inline int get_id_flag(int id_info)
{
    return id_info >> 16;
}

} // namespace transport
#endif // __MASK_HPP__