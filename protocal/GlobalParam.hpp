#ifndef __GLOBAL_PARAM_HPP__
#define __GLOBAL_PARAM_HPP__

#include <atomic>
#include <string>
#include <iostream>
#include <map>
#include <memory>

#ifdef __USE_LIBBASE__
#include "libbase/common.h"
#endif // __USE_LIBBASE__

/************************全局变量定义**********************************/
/***************全局变量需要自行保证全局访问的一致性***********************/
namespace transport
{

/**
 * @brief Flag whether the process is exists or not
 */
struct ProcessExists
{
    std::atomic<bool> exists = true;
};

} // namespace transport



/************************方法实现***********************************/

namespace transport
{
/**
 * @brief 全局变量模板类，通过此类定义和访问全局变量，该方法线程安全
 * 
 * @tparam ParamContent 包含全局变量的结构体或类
 */
template <typename ParamContent>
class GlobalParam
{

protected:
    GlobalParam() = default;
    static ParamContent* m_param;
    static std::mutex m_lock;

	class Deletor {
	public:
		~Deletor() {
			if(GlobalParam<ParamContent>::m_param != nullptr)
				delete GlobalParam<ParamContent>::m_param;
		}
	};
	static Deletor deletor;
public:

    /**
     * @brief 对外接口，负责返回一个用于访问全局变量的指针，函数线程安全
     * 
     * @return ParamContent* 指向全局变量的指针
     */
    static ParamContent* param()
    {
        ParamContent *p = m_param;
        if (p == nullptr)
        {
            std::lock_guard<std::mutex> lock(m_lock);
            if ((p = m_param) == nullptr)
            {
                m_param = p = new ParamContent();
            }
        }
        return p;
    }
};

template <typename ParamContent>
ParamContent* GlobalParam<ParamContent>::m_param = NULL;
template <typename ParamContent>
std::mutex GlobalParam<ParamContent>::m_lock;

/**
 * @brief 提供一种更简短的访问方式
 * 
 */
#define GET_PARAM(ParamContent) transport::GlobalParam<ParamContent>::param()


} // namespace transport

#endif // __GLOBAL_PARAM_HPP__