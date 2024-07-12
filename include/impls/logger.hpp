#ifndef __LOGGER_HPP__
#define __LOGGER_HPP__
#include <iostream>
#include <memory>
#include <string>
#include <mutex>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <cstring>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

#include <sys/stat.h>

// #define __USE_SPD_LOG__
#if defined __USE_ROS2__ && defined __USE_ROS_LOG__ && !defined __NOT_USE_LOG__

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
// #include <rclcpp/init_options.hpp>

#elif defined __USE_SPD_LOG__ && !defined __USE_ROS_LOG__ && !defined __NOT_USE_LOG__

#include <spdlog/spdlog.h>
#include <spdlog/async.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#elif !defined __NOT_USE_LOG__

#include <glog/logging.h>
#include <glog/log_severity.h> 

#else // defined __NOT_USE_LOG__

#endif //__NOT_USE_LOG__

namespace transport{
namespace log{

enum class LOG_SEVERITY
{
	DEBUG,
    INFO, 
    WARNING,
    ERROR,
    FATAL
};

#if defined __USE_ROS2__ && defined __USE_ROS_LOG__ && !defined __NOT_USE_LOG__

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
	int32_t InitROS2Log(rclcpp::Node::SharedPtr node, std::string default_node_name = "__logger__")
	{
		if(m_node == NULL)
		{
			std::lock_guard<std::mutex> lock(m_lock);
			if(node == NULL){
				if(!rclcpp::ok()){
					auto init_options = rclcpp::InitOptions();
					rclcpp::init(0, nullptr, init_options);
				}
				node = std::make_shared<rclcpp::Node>(default_node_name);
			}
			if(m_node == NULL)
				m_node = node;
		}
		return 0;
	}

	int32_t InitROS2Log(std::string default_node_name){
		return InitROS2Log(nullptr, default_node_name);
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

#define LOGINIT(...) transport::log::ROS2Log::Instance()->InitROS2Log(__VA_ARGS__);
#ifdef __DEBUG__
#define LOGDEBUG(...) RCLCPP_DEBUG(transport::log::ROS2Log::Instance()->GetNode()->get_logger(), __VA_ARGS__);
#else 
#define LOGDEBUG(...) ((void)0);
#endif // __DEBUG__
#define LOGINFO(...) RCLCPP_INFO(transport::log::ROS2Log::Instance()->GetNode()->get_logger(), __VA_ARGS__);
#define LOGWARN(...) RCLCPP_WARN(transport::log::ROS2Log::Instance()->GetNode()->get_logger(), __VA_ARGS__);
#define LOGERROR(...) RCLCPP_ERROR(transport::log::ROS2Log::Instance()->GetNode()->get_logger(), __VA_ARGS__);
#define LOGFATAL(...) RCLCPP_FATAL(transport::log::ROS2Log::Instance()->GetNode()->get_logger(), __VA_ARGS__);


#elif defined __USE_SPD_LOG__ && !defined __USE_ROS_LOG__ && !defined __NOT_USE_LOG__

class SpdLog
{
private:
    SpdLog(){};

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
		std::filesystem::path path(logs_folder_dir);
		if(!std::filesystem::exists(path))
		{
			std::filesystem::create_directories(path);
		}

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

	~SpdLog()
	{
		// spdlog::drop_all();
		// m_logger->drop();
		spdlog::drop(m_logger->name());
	};
 
    /**
     * @brief 初始化spdlog
     * 
     * @param name logger名
     * @param log_dir log文件存储路径
     * @return int32_t 无
     */
	int32_t InitSpdLog(const char * name = "transport", const char * log_dir = "./log/")
	{
		if(m_logger == NULL)
		{
			std::lock_guard<std::mutex> lock(m_lock);
			if(m_logger == NULL){
				std::string _log_dir;
				if(CreateLogDirectory(_log_dir, log_dir, name))
				{
					spdlog::init_thread_pool(8192, 1);
					auto rotat_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(_log_dir + "/log.txt", 1024 * 1024 * 1, 5); // 滚动日志，单文件最大1M，最多5文件
					auto std_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>(spdlog::color_mode::automatic);
					std::vector<spdlog::sink_ptr> sinks = {rotat_sink, std_sink};
					m_logger = std::make_shared<spdlog::async_logger>(std::string(name), sinks.begin(), sinks.end(), spdlog::thread_pool(), spdlog::async_overflow_policy::overrun_oldest);
#ifdef __DEBUG__
					m_logger->set_level(spdlog::level::debug);
#endif // __DEBUG__
					std::clog << "spd_log_dir: " << _log_dir << std::endl;
				}
				else
				{
					spdlog::init_thread_pool(8192, 1);
					auto rotat_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>("./log.txt", 1024 * 1024 * 1, 5); // 滚动日志，单文件最大1M，最多5文件
					auto std_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>(spdlog::color_mode::automatic);
					std::vector<spdlog::sink_ptr> sinks = {rotat_sink, std_sink};
					m_logger = std::make_shared<spdlog::async_logger>(std::string(name), sinks.begin(), sinks.end(), spdlog::thread_pool(), spdlog::async_overflow_policy::overrun_oldest);
#ifdef __DEBUG__
					m_logger->set_level(spdlog::level::debug);
#endif // __DEBUG__
					std::clog << "spd_log_dir: " << "CreateLogDirectory failed, use default log dir './log.txt'" << std::endl;
				}
			}
		}
		return 0;
	}
 
    /**
     * @brief 获取logger
     * 
     * @return std::shared_ptr<spdlog::logger> 
     */
	std::shared_ptr<spdlog::logger> GetLogger()
	{
		return m_logger;		
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
	void SpdLogMsg(const char* __file__, int __line__, LOG_SEVERITY severity, const char *format, ...)
	{

        char buffer[1024] = {0};
		va_list arg_ptr;
		va_start(arg_ptr, format);
		vsprintf(buffer, format, arg_ptr);
		va_end(arg_ptr);	

		if(!m_logger){
			return;
		}
		switch(severity)
		{
		case LOG_SEVERITY::DEBUG:
			m_logger->debug("[{}:{}]: {}", __file__, __line__, buffer);
			break;
		case LOG_SEVERITY::INFO:
			m_logger->info("[{}:{}]: {}", __file__, __line__, buffer);
			break;
		case LOG_SEVERITY::WARNING:
			m_logger->warn("[{}:{}]: {}", __file__, __line__, buffer);
			break;
		case LOG_SEVERITY::ERROR:
			m_logger->error("[{}:{}]: {}", __file__, __line__, buffer);
			break;
		case LOG_SEVERITY::FATAL:
			m_logger->error("[{}:{}]: {}", __file__, __line__, buffer); // spdlog 没有 fatal
			break;
		default:
			break;
		}		
			
	}

	void SpdLogMsg(const char* __file__, int __line__, LOG_SEVERITY severity, std::string format, ...)
	{
		char buffer[1024] = {0};
		va_list arg_ptr;
		// va_start(arg_ptr, format.c_str());
		va_start(arg_ptr, format);
		vsprintf(buffer, format.c_str(), arg_ptr);
		va_end(arg_ptr);	

		if(!m_logger){
			return;
		}
		switch(severity)
		{
		case LOG_SEVERITY::DEBUG:
			m_logger->debug("[{}:{}]: {}", __file__, __line__, buffer);
			break;
		case LOG_SEVERITY::INFO:
			m_logger->info("[{}:{}]: {}", __file__, __line__, buffer);
			break;
		case LOG_SEVERITY::WARNING:
			m_logger->warn("[{}:{}]: {}", __file__, __line__, buffer);
			break;
		case LOG_SEVERITY::ERROR:
			m_logger->error("[{}:{}]: {}", __file__, __line__, buffer);
			break;
		case LOG_SEVERITY::FATAL:
			m_logger->error("[{}:{}]: {}", __file__, __line__, buffer); // spdlog 没有 fatal
			break;
		default:
			break;
		}			
	}

public:
	static SpdLog * Instance()
	{
		if (!m_pInstance)
		{
            std::lock_guard<std::mutex> lock(m_lock);
            if(!m_pInstance)
			    m_pInstance = std::unique_ptr<SpdLog>(new SpdLog());
		}
		return m_pInstance.get();
	}

private:
    static std::unique_ptr<SpdLog> m_pInstance;
    static std::mutex m_lock;
	std::shared_ptr<spdlog::logger> m_logger = NULL;
};

std::unique_ptr<SpdLog> SpdLog::m_pInstance = nullptr;
std::mutex SpdLog::m_lock;

#define LOGINIT(...) transport::log::SpdLog::Instance()->InitSpdLog(__VA_ARGS__);
#ifdef __DEBUG__
#define LOGDEBUG(...) transport::log::SpdLog::Instance()->SpdLogMsg(__FILE__, __LINE__, transport::log::LOG_SEVERITY::DEBUG, __VA_ARGS__);
#else 
#define LOGDEBUG(...) ((void)0);
#endif // __DEBUG__
#define LOGINFO(...) transport::log::SpdLog::Instance()->SpdLogMsg(__FILE__, __LINE__, transport::log::LOG_SEVERITY::INFO, __VA_ARGS__);
#define LOGWARN(...) transport::log::SpdLog::Instance()->SpdLogMsg(__FILE__, __LINE__, transport::log::LOG_SEVERITY::WARNING, __VA_ARGS__);
#define LOGERROR(...) transport::log::SpdLog::Instance()->SpdLogMsg(__FILE__, __LINE__, transport::log::LOG_SEVERITY::ERROR, __VA_ARGS__);
#define LOGFATAL(...) transport::log::SpdLog::Instance()->SpdLogMsg(__FILE__, __LINE__, transport::log::LOG_SEVERITY::FATAL, __VA_ARGS__);

#elif !defined __NOT_USE_LOG__ 

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
		std::filesystem::path path(logs_folder_dir);
		if(!std::filesystem::exists(path))
		{
			std::filesystem::create_directories(path);
		}

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
		case LOG_SEVERITY::DEBUG:
			google::LogMessage(__file__, __line__, google::GLOG_INFO).stream() << buffer;	// glog没有debug级别
			break;
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
		// va_start(arg_ptr, format.c_str());
		va_start(arg_ptr, format);
		vsprintf(buffer, format.c_str(), arg_ptr);
		va_end(arg_ptr);	

		switch(severity)
		{
		case LOG_SEVERITY::DEBUG:
			google::LogMessage(__file__, __line__, google::GLOG_INFO).stream() << buffer;	// glog没有debug级别
			break;
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
		if (!m_pInstance)
		{
            std::lock_guard<std::mutex> lock(m_lock);
            if(!m_pInstance)
			    m_pInstance = std::unique_ptr<GLog>(new GLog());
		}
		return m_pInstance.get();
	}
 
private:
	static std::unique_ptr<GLog> m_pInstance;
    static std::mutex m_lock;
};
 
std::unique_ptr<GLog> GLog::m_pInstance = nullptr;
std::mutex GLog::m_lock;

#define LOGINIT(...) transport::log::GLog::Instance()->InitGLog(__VA_ARGS__);
#ifdef __DEBUG__
#define LOGDEBUG(...) transport::log::GLog::Instance()->GLogMsg(__FILE__, __LINE__, transport::log::LOG_SEVERITY::DEBUG, __VA_ARGS__);
#else 
#define LOGDEBUG(...) ((void)0);
#endif // __DEBUG__
#define LOGINFO(...) transport::log::GLog::Instance()->GLogMsg(__FILE__, __LINE__, transport::log::LOG_SEVERITY::INFO, __VA_ARGS__);
#define LOGWARN(...) transport::log::GLog::Instance()->GLogMsg(__FILE__, __LINE__, transport::log::LOG_SEVERITY::WARNING, __VA_ARGS__);
#define LOGERROR(...) transport::log::GLog::Instance()->GLogMsg(__FILE__, __LINE__, transport::log::LOG_SEVERITY::ERROR, __VA_ARGS__);
#define LOGFATAL(...) transport::log::GLog::Instance()->GLogMsg(__FILE__, __LINE__, transport::log::LOG_SEVERITY::FATAL, __VA_ARGS__);

#else // __NOT_USE_LOG__
#pragma message("UNUSE Log")

#define LOGINIT(...) ((void)0);
#define LOGDEBUG(...) ((void)0);
#define LOGINFO(...) ((void)0);
#define LOGWARN(...) ((void)0);
#define LOGERROR(...) ((void)0);
#define LOGFATAL(...) ((void)0);

#endif // !define __NOT_USE_LOG__

} // namespace log

} // namespace transport
#endif // __LOGGER_HPP__