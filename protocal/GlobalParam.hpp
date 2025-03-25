#ifndef __GLOBAL_PARAM_HPP__
#define __GLOBAL_PARAM_HPP__

#include <mutex>
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
    static std::unique_ptr<ParamContent> m_param;
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
        if (!m_param)
        {
            std::lock_guard<std::mutex> lock(m_lock);
            if (!m_param)
            {
                m_param = std::make_unique<ParamContent>();
            }
        }
        return m_param.get();
    }
};

template <typename ParamContent>
std::unique_ptr<ParamContent> GlobalParam<ParamContent>::m_param = nullptr;
template <typename ParamContent>
std::mutex GlobalParam<ParamContent>::m_lock;

/**
 * @brief 提供一种更简短的访问方式
 * 
 */
#define GET_PARAM(ParamContent) transport::GlobalParam<ParamContent>::param()


} // namespace transport

#endif // __GLOBAL_PARAM_HPP__