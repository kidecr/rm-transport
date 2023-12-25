#ifndef __GLOBAL_PARAM_HPP__
#define __GLOBAL_PARAM_HPP__

#include <string>
#include <iostream>

#ifdef __USE_LIBBASE__
#include "libbase/common.h"
#endif // __USE_LIBBASE__

namespace transport
{

/**
 * @brief Flag whether the process is exists or not
 */
struct ProcessExists
{
    bool exists = true;
}

} // namespace transport

#endif // __GLOBAL_PARAM_HPP__