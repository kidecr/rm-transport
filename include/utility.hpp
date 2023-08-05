#ifndef __UTILITY_HPP__
#define __UTILITY_HPP__

#include <sched.h>

#ifndef PORT_ASSERT
#define PORT_ASSERT( expr ) do { if(!!(expr)) ; else throw new PortException(#expr, __func__, __FILE__, __LINE__ ); } while(0)
#endif // PORT_ASSERT

class PortException : public std::exception
{
public:
    PortException(std::string message) { this->message = message; };
    PortException(std::string info, std::string func, std::string file, std::string line) {
        this->message = file + ":" + line + ": in " + func + " " + info; 
    }
    PortException(const char* info, const char* func, const char* file, int line) {
        this->message = std::string(file) + ":" + std::to_string(line) + ": in " + std::string(func) + " " + std::string(info); 
    }
    ~PortException(){};
    std::string message;
    const char* what()
    {
        if (message.empty())
            std::cout << "empty" << std::endl;
        return message.c_str();
    }
};



/**
 * @brief 设置线程内核绑定(亲和度)
 * 
 * @param cpu_id 目标内核
 */
void set_cpu_affinity(int cpu_id) {
    cpu_set_t set;
    CPU_ZERO(&set);
    // 设置绑定的核
    CPU_SET(cpu_id, &set);
    // 设置绑定当前进程
    sched_setaffinity(0, sizeof(cpu_set_t), &set);
}

#endif // __UTILITY_HPP__