#ifndef __LOGGER_HPP__
#define __LOGGER_HPP__
#include <iostream>
#include <memory>
#include <string>
#include <mutex>
#include <chrono>
#include <iomanip>
#include <cstring>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

#include <sys/stat.h>

#if defined __USE_ROS2__ && defined __USE_ROS_LOG__

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

#else

#include <glog/logging.h>
#include <glog/log_severity.h> 

#endif //__USE_ROS2__ && __USE_ROS_LOG__

namespace transport{
namespace log{

enum class LOG_SEVERITY
{
    INFO, 
    WARNING,
    ERROR,
    FATAL
};

#if defined __USE_ROS2__ && defined __USE_ROS_LOG__

class ROS2Log
{
private:
    ROS2Log(){};

public:

	~ROS2Log(){};
 
    /**
     * @brief 初始化GLOG参数
     * 
     * @param name Log程序名
     * @param log_dir Log存储路径
     * @return int32_t 无
     */
	int32_t InitROS2Log(rclcpp::Node::SharedPtr node)
	{
		if(m_node == NULL)
		{
			std::lock_guard<std::mutex> lock(m_lock);
			if(node == NULL)
				node = std::make_shared<rclcpp::Node>("__logger__");
			if(m_node == NULL)
				m_node = node;
		}
		return 0;
	}
 
    /**
     * @brief 获取node节点
     * 
     * @return rclcpp::Node::SharedPtr 
     */
	rclcpp::Node::SharedPtr GetNode()
	{
		return m_node;		
	}

public:
	static ROS2Log * Instance()
	{
		if (NULL == m_pInstance)
		{
            std::lock_guard<std::mutex> lock(m_lock);
            if(NULL == m_pInstance)
			    m_pInstance = new ROS2Log();
		}
		return m_pInstance;
	}

private:
    static ROS2Log * m_pInstance;
    static std::mutex m_lock;
	rclcpp::Node::SharedPtr m_node = NULL;
};

ROS2Log * ROS2Log::m_pInstance = NULL;
std::mutex ROS2Log::m_lock;

#define LOGINIT(Node) transport::log::ROS2Log::Instance()->InitROS2Log(Node);
#define LOGINFO(...) RCLCPP_INFO(transport::log::ROS2Log::Instance()->GetNode()->get_logger(), __VA_ARGS__);
#define LOGWARN(...) RCLCPP_WARN(transport::log::ROS2Log::Instance()->GetNode()->get_logger(), __VA_ARGS__);
#define LOGERROR(...) RCLCPP_ERROR(transport::log::ROS2Log::Instance()->GetNode()->get_logger(), __VA_ARGS__);
#define LOGFATAL(...) RCLCPP_FATAL(transport::log::ROS2Log::Instance()->GetNode()->get_logger(), __VA_ARGS__);


#else

class GLog
{
private:
	GLog(void){}

		/**
	 * @brief 创建当前程序的log路径
	 * 
	 * @param log_dir 当前程序的log路径
	 * @param logs_folder_dir 设定的log路径
	 * @return true 创建成功
	 * @return false 创建失败
	 */
	bool CreateLogDirectory(std::string &log_dir, const char* logs_folder_dir, const char* name)
	{
		createDirectory(logs_folder_dir);

		auto t = time(NULL);
		std::stringstream date;
		std::string _log_folder_dir = logs_folder_dir;
		if(_log_folder_dir.back() != '/') 
			_log_folder_dir.push_back('/');
		date << _log_folder_dir << name << "_" << std::put_time(std::localtime(&t), "%Y-%m-%d_%X");
		log_dir = date.str();
		if(mkdir(log_dir.c_str(), S_IRWXU) != 0)
		{
			std::clog << __FILE__ << ":" << __LINE__ << ":  create dir '" << log_dir << "' failed" << std::endl;
			return false;
		}
		return true;
	}
public:
	~GLog(){};
 
    /**
     * @brief 初始化GLOG参数
     * 
     * @param name Log程序名
     * @param log_dir Log存储路径
     * @return int32_t 无
     */
	int32_t InitGLog(const char * name = "transport", const char * log_dir = "./log/")
	{
		google::InitGoogleLogging(name);
		std::string _log_dir;
        if(CreateLogDirectory(_log_dir, log_dir, name))
		{
			FLAGS_log_dir = _log_dir;
			std::cout << "FLAGS_log_dir: " << FLAGS_log_dir << std::endl;
		}
		FLAGS_alsologtostderr = true;
        FLAGS_colorlogtostderr = true;
		FLAGS_max_log_size = 1;	// MB
 
		return 0;
	}
 
    /**
     * @brief 输出一条LOG信息
     * 
     * @param __file__ 输入__FILE__
     * @param __line__ 输入__LINE__
     * @param severity 输入日志等级
     * @param format 格式化字符串
     * @param ... 参数
     */
	void GLogMsg(const char* __file__, int __line__, LOG_SEVERITY severity, const char *format, ...)
	{
        char buffer[1024] = {0};
		va_list arg_ptr;
		va_start(arg_ptr, format);
		vsprintf(buffer, format, arg_ptr);
		va_end(arg_ptr);	

		switch(severity)
		{
		case LOG_SEVERITY::INFO:
			google::LogMessage(__file__, __line__, google::GLOG_INFO).stream() << buffer;
			break;
		case LOG_SEVERITY::WARNING:
			google::LogMessage(__file__, __line__, google::GLOG_WARNING).stream() << buffer;
			break;
		case LOG_SEVERITY::ERROR:
			google::LogMessage(__file__, __line__, google::GLOG_ERROR).stream() << buffer;
			break;
		case LOG_SEVERITY::FATAL:
			google::LogMessage(__file__, __line__, google::GLOG_FATAL).stream() << buffer;
			break;
		default:
			break;
		}		
			
	}

	void GLogMsg(const char* __file__, int __line__, LOG_SEVERITY severity, std::string format, ...)
	{
		char buffer[1024] = {0};
		va_list arg_ptr;
		va_start(arg_ptr, format.c_str());
		vsprintf(buffer, format.c_str(), arg_ptr);
		va_end(arg_ptr);	

		switch(severity)
		{
		case LOG_SEVERITY::INFO:
			google::LogMessage(__file__, __line__, google::GLOG_INFO).stream() << buffer;
			break;
		case LOG_SEVERITY::WARNING:
			google::LogMessage(__file__, __line__, google::GLOG_WARNING).stream() << buffer;
			break;
		case LOG_SEVERITY::ERROR:
			google::LogMessage(__file__, __line__, google::GLOG_ERROR).stream() << buffer;
			break;
		case LOG_SEVERITY::FATAL:
			google::LogMessage(__file__, __line__, google::GLOG_FATAL).stream() << buffer;
			break;
		default:
			break;
		}		
	}
 
public:
	static GLog * Instance()
	{
		if (NULL == m_pInstance)
		{
            std::lock_guard<std::mutex> lock(m_lock);
            if(NULL == m_pInstance)
			    m_pInstance = new GLog();
		}
		return m_pInstance;
	}
 
private:
	static GLog * m_pInstance;
    static std::mutex m_lock;
};
 
GLog * GLog::m_pInstance = NULL;
std::mutex GLog::m_lock;

#define LOGINIT(...) transport::log::GLog::Instance()->InitGLog(__VA_ARGS__);
#define LOGINFO(...) transport::log::GLog::Instance()->GLogMsg(__FILE__, __LINE__, transport::log::LOG_SEVERITY::INFO, __VA_ARGS__);
#define LOGWARN(...) transport::log::GLog::Instance()->GLogMsg(__FILE__, __LINE__, transport::log::LOG_SEVERITY::WARNING, __VA_ARGS__);
#define LOGERROR(...) transport::log::GLog::Instance()->GLogMsg(__FILE__, __LINE__, transport::log::LOG_SEVERITY::ERROR, __VA_ARGS__);
#define LOGFATAL(...) transport::log::GLog::Instance()->GLogMsg(__FILE__, __LINE__, transport::log::LOG_SEVERITY::FATAL, __VA_ARGS__);

#endif // //__USE_ROS2__ && __USE_ROS_LOG__

} // namespace log

} // namespace transport
#endif // __LOGGER_HPP__