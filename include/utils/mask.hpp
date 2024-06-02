#ifndef __MASK_HPP__
#define __MASK_HPP__

#include "protocal/Protocal.hpp"
#include "protocal/IDs.hpp"

namespace transport{

/**********************************************************
 * 程序索引ID时ID结构定义:
 * ID作为统一CAN和SERIAL以及未来可能会出现的其他类型包的标识，一定是由设备标识和ID共同组成的。
 * 而CAN_ID和SERIAL_ID等均为未编码的。
 * ID结构：0000 0000 0000 0000 0000 0000 0000 0000
 *       |         |         |                   |
 *           保留     设备标识          id
 */

/**
 * @brief 添加CAN_ID设备标识0x01
 * 
 * @param can_id 
 * @return ID 
 */
inline ID mask(CAN_ID can_id)
{
    int id = static_cast<int>(can_id);
    id = id | 0x010000; // 添加can标识符
    return static_cast<ID>(id);
}

/**
 * @brief 添加SERIAL_ID设备标识0x02
 * 
 * @param can_id 
 * @return ID 
 */
inline ID mask(SERIAL_ID can_id)
{
    int id = static_cast<int>(can_id);
    id = id | 0x020000; // 添加serial标识符
    return static_cast<ID>(id);
}

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