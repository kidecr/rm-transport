#ifndef __ERROE_HPP__
#define __ERROE_HPP__

#include <exception>
#include <string>
#include <iostream>

namespace transport{

class PortException : public std::exception
{
public:
    PortException(std::string message) { this->message = message; };
    PortException(std::string info, std::string func, std::string file, std::string line) {
        this->message = file + ":" + line + ": in function " + func + " : " + info; 
    }
    PortException(const char* info, const char* func, const char* file, int line) {
        this->message = std::string(file) + ":" + std::to_string(line) + ": in function " + std::string(func) + " : " + std::string(info); 
    }
    PortException(std::string info, const char* func, const char* file, int line) {
        this->message = std::string(file) + ":" + std::to_string(line) + ": in function " + std::string(func) + " : " + std::string(info); 
    }

    ~PortException(){};
    std::string message;
    const char* what() const noexcept override
    {
        if (message.empty())
            std::cout << "empty" << std::endl;
        return message.c_str();
    }
};

} // namespace transport

#ifndef PORT_ASSERT // 如果判断内容为true 则通过，否则抛出异常
#define PORT_ASSERT( expr ) do { if(!!(expr)) ; else throw transport::PortException(#expr, __PRETTY_FUNCTION__, __FILE__, __LINE__ ); } while(0)
#endif // PORT_ASSERT

#ifndef PORT_EXCEPTION
#define PORT_EXCEPTION( msg ) transport::PortException(( msg ), __PRETTY_FUNCTION__, __FILE__, __LINE__ )
#endif // PORT_EXCEPTION

#endif // __ERROE_HPP__