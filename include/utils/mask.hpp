#ifndef __MASK_HPP__
#define __MASK_HPP__

#include "protocal/Protocal.hpp"

namespace transport{


inline ID mask(CAN_ID can_id)
{
    int id = static_cast<int>(can_id);
    id = id | 0x010000; // 添加can标识符
    return static_cast<ID>(id);
}

inline ID mask(SERIAL_ID can_id)
{
    int id = static_cast<int>(can_id);
    id = id | 0x020000; // 添加serial标识符
    return static_cast<ID>(id);
}

inline ID mask(int id)
{
    return static_cast<ID>(id);
}

inline int unmask(ID id)
{
    int masked_id = static_cast<int>(id);
    return masked_id & 0x00ffff;
}

} // namespace transport
#endif // __MASK_HPP__