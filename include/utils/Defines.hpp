#ifndef __DEFINES_HPP__
#define __DEFINES_HPP__

#if defined(__GNUC__) || defined(__clang__)
#include <cxxabi.h>
#endif

#ifndef __CLASS__
// 用于类内，内容为类名，string类型
#if defined(__GNUC__) || defined(__clang__)
#define __CLASS__ std::unique_ptr<char>(abi::__cxa_demangle(typeid(*this).name(), 0, 0, 0)).get()
#else // win
#define __CLASS__ typeid(*this).name()
#endif
#endif // __CLASS__

#ifndef __TYPE
// 内容为参数type的名，string类型
#if defined(__GNUC__) || defined(__clang__)
#define __TYPE(type) std::unique_ptr<char>(abi::__cxa_demangle(typeid(type).name(), 0, 0, 0)).get()
#else // win
#define __TYPE(type) typeid(type).name()
#endif
#endif // __TYPE()

#if defined(_MSC_VER)  // MSVC
#define PRETTY_FUNCTION __FUNCSIG__
#elif defined(__clang__) || defined(__GNUC__)  // Clang/GCC
#define PRETTY_FUNCTION __PRETTY_FUNCTION__
#else
#error "Unsupported compiler"
#endif

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
#define HINT LOGINFO("\033[93mHINT\033[0m")
#endif // HINT

#endif // __DEFINES_HPP__

