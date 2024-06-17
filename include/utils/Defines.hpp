#ifndef __DEFINES_HPP__
#define __DEFINES_HPP__

#include <cxxabi.h>

#ifndef __CLASS__
// 用于类内，内容为类名，string类型
#define __CLASS__ (abi::__cxa_demangle(typeid(*this).name(), 0, 0, 0))
#endif // __CLASS__

#ifndef __TYPE
// 内容为参数type的名，string类型
#define __TYPE(type) (abi::__cxa_demangle(typeid(type).name(), 0, 0, 0))
#endif // __TYPE()

#ifndef PI
#define PI 3.14159265358979323846
#endif // PI

#ifndef TO_STR
#define TO_STR(var) #var << ": " << var
#endif // TO_STR

// IENUM和UENUM是两个不会占用内存空间的常量类型，有作用域，用于解决ENUM没有作用域而class ENUM没发直接转int的问题
#ifndef IENUM
#define IENUM static constexpr int 
#endif // IENUM

#ifndef UENUM
#define UENUM static constexpr unsigned int 
#endif // UENUM

#ifndef HINT
#define HINT LOGDEBUG("\033[93mHINT\033[0m")
#endif // HINT

#endif // __DEFINES_HPP__

