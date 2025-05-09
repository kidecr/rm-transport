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

// #include <sys/stat.h>

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
#define GLOG_USE_GLOG_EXPORT // glog版本大于0.7时要加这个
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


#elif defined __USE_SPD_LOG__ && !defined __NOT_USE_LOG__

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
	bool createLogDirectory(std::string& log_dir, const char* logs_folder_dir, const char* name) {
		using namespace std::chrono;
		namespace fs = std::filesystem;

		fs::path path(logs_folder_dir);
		// 根据当前时间组和得到文件夹名
		auto now = system_clock::to_time_t(system_clock::now());
		std::stringstream date_stream;
		date_stream << name << "_" << std::put_time(std::localtime(&now), "%Y-%m-%d_%X");

		// 得到log路径
		path = path / date_stream.str();
		
		// 创建路径
		if (!fs::exists(path)) {
			if (fs::create_directories(path)) {
				log_dir = path.string();
				return true;
			}
			else{
				std::clog << "Failed to create directory: " << path << std::endl;
				return false;
			}
		} else {
			// 路径已经存在，在该目录下创建子文件夹，并以1 2 3 4命名
			for (int i = 1; i <= 100; ++i) {
				fs::path sub_path = path / std::to_string(i);
				if (!fs::exists(sub_path)) {
					if (!fs::create_directory(sub_path)) {
						std::clog << "Failed to create subdirectory: " << sub_path << std::endl;
						return false;
					}
					log_dir = sub_path.string();
					return true;
				}
			}
			
			// Failed to create any subdirectory after trying up to 100 times
			std::clog << "Unable to create unique subdirectory under: " << path << std::endl;
			return false;
		}
		log_dir = path.string();
		return true;
	}
public:

	~SpdLog()
	{
		// spdlog::drop_all();
		// m_logger->drop();
		// spdlog::drop(m_logger->name());
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
				if(createLogDirectory(_log_dir, log_dir, name))
				{
					spdlog::init_thread_pool(8192, 1);
					auto info_rotat_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(_log_dir + "/log.INFO", 1024 * 1024 * 1, 5); // 滚动日志，单文件最大1M，最多5文件
                    auto warn_rotat_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(_log_dir + "/log.WARN", 1024 * 1024 * 1, 5); // 滚动日志，单文件最大1M，最多5文件
                    auto error_rotat_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(_log_dir + "/log.ERROR", 1024 * 1024 * 1, 5); // 滚动日志，单文件最大1M，最多5文件
					auto std_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>(spdlog::color_mode::automatic);
                    info_rotat_sink->set_level(spdlog::level::info);
                    warn_rotat_sink->set_level(spdlog::level::warn);
                    error_rotat_sink->set_level(spdlog::level::err);
#ifndef __DEBUG__
					std::vector<spdlog::sink_ptr> sinks = {info_rotat_sink, warn_rotat_sink, error_rotat_sink, std_sink};
#else
                    auto debug_rotat_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(_log_dir + "/log.DEBUG", 1024 * 1024 * 1, 10); // 滚动日志，单文件最大1M，最多10文件
					debug_rotat_sink->set_level(spdlog::level::debug);
                    std::vector<spdlog::sink_ptr> sinks = {debug_rotat_sink, info_rotat_sink, warn_rotat_sink, error_rotat_sink, std_sink};
#endif // __DEBUG__
					m_logger = std::make_shared<spdlog::async_logger>(std::string(name), sinks.begin(), sinks.end(), spdlog::thread_pool(), spdlog::async_overflow_policy::overrun_oldest);
					m_logger->info("spd_log_dir: {}", _log_dir);
				}
				else
				{
					spdlog::init_thread_pool(8192, 1);
                    auto info_rotat_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>("./log.INFO", 1024 * 1024 * 1, 5); // 滚动日志，单文件最大1M，最多5文件
                    auto warn_rotat_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>("./log.WARN", 1024 * 1024 * 1, 5); // 滚动日志，单文件最大1M，最多5文件
                    auto error_rotat_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>("./log.ERROR", 1024 * 1024 * 1, 5); // 滚动日志，单文件最大1M，最多5文件
					auto std_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>(spdlog::color_mode::automatic);
                    info_rotat_sink->set_level(spdlog::level::info);
                    warn_rotat_sink->set_level(spdlog::level::warn);
                    error_rotat_sink->set_level(spdlog::level::err);
#ifndef __DEBUG__
					std::vector<spdlog::sink_ptr> sinks = {info_rotat_sink, warn_rotat_sink, error_rotat_sink, std_sink};
#else
                    auto debug_rotat_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>("./log.DEBUG", 1024 * 1024 * 1, 10); // 滚动日志，单文件最大1M，最多10文件
					debug_rotat_sink->set_level(spdlog::level::debug);
                    std::vector<spdlog::sink_ptr> sinks = {debug_rotat_sink, info_rotat_sink, warn_rotat_sink, error_rotat_sink, std_sink};
#endif // __DEBUG__
					m_logger->warn("spd_log_dir: CreateLogDirectory failed, use default log dir './log.*'");
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

#elif defined __USE_G_LOG__ && !defined __NOT_USE_LOG__ 

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
	bool createLogDirectory(std::string& log_dir, const char* logs_folder_dir, const char* name) {
		using namespace std::chrono;
		namespace fs = std::filesystem;

		fs::path path(logs_folder_dir);
		// 根据当前时间组和得到文件夹名
		auto now = system_clock::to_time_t(system_clock::now());
		std::stringstream date_stream;
		date_stream << name << "_" << std::put_time(std::localtime(&now), "%Y-%m-%d_%H-%M-%S");

		// 得到log路径
		path = path / date_stream.str();
		
		// 创建路径
		if (!fs::exists(path)) {
			if (fs::create_directories(path)) {
				log_dir = path.string();
				return true;
			}
			else{
				std::clog << "Failed to create directory: " << path << std::endl;
				return false;
			}
		} else {
			// 路径已经存在，在该目录下创建子文件夹，并以1 2 3 4命名
			for (int i = 1; i <= 100; ++i) {
				fs::path sub_path = path / std::to_string(i);
				if (!fs::exists(sub_path)) {
					if (!fs::create_directory(sub_path)) {
						std::clog << "Failed to create subdirectory: " << sub_path << std::endl;
						return false;
					}
					log_dir = sub_path.string();
					return true;
				}
			}
			
			// Failed to create any subdirectory after trying up to 100 times
			std::clog << "Unable to create unique subdirectory under: " << path << std::endl;
			return false;
		}
		log_dir = path.string();
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
		if (!google::IsGoogleLoggingInitialized())
			google::InitGoogleLogging(name);
		std::string _log_dir;
        if(createLogDirectory(_log_dir, log_dir, name))
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

#elif defined __NOT_USE_LOG__
#pragma message("UNUSE Log")

#define LOGINIT(...) ((void)0);
#define LOGDEBUG(...) ((void)0);
#define LOGINFO(...) ((void)0);
#define LOGWARN(...) ((void)0);
#define LOGERROR(...) ((void)0);
#define LOGFATAL(...) ((void)0);

#else

class StdLog {
private:
	StdLog() = default;

public:
	~StdLog() = default;

	/**
	 * @brief 输出一条日志信息
	 * 
	 * @param __file__ 文件名
	 * @param __line__ 行号
	 * @param severity 日志级别
	 * @param format 格式化字符串
	 * @param ... 参数
	 */
	void LogMsg(const char* __file__, int __line__, LOG_SEVERITY severity, const char* format, ...) {
		std::ostringstream oss;
		va_list args;

		// 获取当前时间
		auto now = std::time(nullptr);
		char time_str[20];
		std::strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", std::localtime(&now));

		// 构造日志头
		oss << "[" << time_str << "] ";
		switch (severity) {
			case LOG_SEVERITY::DEBUG: oss << "[DEBUG] "; break;
			case LOG_SEVERITY::INFO: oss << "[INFO] "; break;
			case LOG_SEVERITY::WARNING: oss << "[WARNING] "; break;
			case LOG_SEVERITY::ERROR: oss << "[ERROR] "; break;
			case LOG_SEVERITY::FATAL: oss << "[FATAL] "; break;
		}
		oss << __file__ << ":" << __line__ << " - ";

		// 格式化日志内容
		va_start(args, format);
		char buffer[1024];
		vsnprintf(buffer, sizeof(buffer), format, args);
		va_end(args);

		oss << buffer;

		// 防止输出乱掉
		std::lock_guard<std::mutex> lock(m_lock);
		// 输出日志
		if (severity == LOG_SEVERITY::FATAL || severity == LOG_SEVERITY::ERROR) {
			std::cerr << oss.str() << std::endl;
		} else {
			std::cout << oss.str() << std::endl;
		}
	}

	/**
	 * @brief 单例模式获取实例
	 * 
	 * @return StdLog* 实例指针
	 */
	static StdLog* Instance() {
		static StdLog instance;
        return &instance;
	}

private:
	static std::mutex m_lock;
};

std::mutex StdLog::m_lock;

// 宏定义简化日志调用
#define LOGINIT(...) ((void)0)  // 初始化无意义，这里省略
#ifdef __DEBUG__
#define LOGDEBUG(...) transport::log::StdLog::Instance()->LogMsg(__FILE__, __LINE__, transport::log::LOG_SEVERITY::DEBUG, __VA_ARGS__);
#else
#define LOGDEBUG(...) ((void)0);
#endif  // __DEBUG__
#define LOGINFO(...) transport::log::StdLog::Instance()->LogMsg(__FILE__, __LINE__, transport::log::LOG_SEVERITY::INFO, __VA_ARGS__);
#define LOGWARN(...) transport::log::StdLog::Instance()->LogMsg(__FILE__, __LINE__, transport::log::LOG_SEVERITY::WARNING, __VA_ARGS__);
#define LOGERROR(...) transport::log::StdLog::Instance()->LogMsg(__FILE__, __LINE__, transport::log::LOG_SEVERITY::ERROR, __VA_ARGS__);
#define LOGFATAL(...) transport::log::StdLog::Instance()->LogMsg(__FILE__, __LINE__, transport::log::LOG_SEVERITY::FATAL, __VA_ARGS__);

#endif // !define __NOT_USE_LOG__

} // namespace log

} // namespace transport
#endif // __LOGGER_HPP__