#ifndef __MASK_HPP__
#define __MASK_HPP__

#include "protocal/Protocal.hpp"

ID mask(CAN_ID& can_id)
{
    int id = static_cast<int>(can_id);
    id = id | 0x010000; // 添加can标识符
    return static_cast<ID>(id);
}

ID mask(SERIAL_ID& can_id)
{
    int id = static_cast<int>(can_id);
    id = id | 0x020000; // 添加serial标识符
    return static_cast<ID>(id);
}

#endif // __MASK_HPP__