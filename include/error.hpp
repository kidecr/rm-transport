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
    const char* what()
    {
        if (message.empty())
            std::cout << "empty" << std::endl;
        return message.c_str();
    }
};

} // namespace transport

#endif // __ERROE_HPP__