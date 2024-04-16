#include <cstdint>
#include <string>
#include <vector>
#include <iostream>

#include "protocal/Protocal.hpp"
#include "impls/logger.hpp"
#include "PackageManager.hpp"
#include "PortManager.hpp"

enum DEVICE_TYPE{
    NONE,
    SERIAL,
    CAN
}
struct ArgsParser
{
    DEVICE_TYPE device_type;
    int32_t id;
};

ArgsParser argsParser(int argc, char* argv[])
{
    ArgsParser args_parser;
    if( argc != 5 
        || std::string(argv[1]) != "--device-type"
        || std::string(argv[3]) != "--id")
    {
        std::cout << "输入参数错误" << std::endl;
        std::cout << "RecvPackage --device-type serial|can --id [id]" << std::endl;
        args_parser.device_type = DEVICE_TYPE::NONE;
        args_parser.id = -1;
        return args_parser;
    }
    std::string device_type(argv[2]);
    if(device_type == "serial")
        args_parser.device_type = DEVICE_TYPE::SERIAL;
    else if (device_type == "can")
        args_parser.device_type = DEVICE_TYPE::CAN;
    else {
        args_parser.device_type = DEVICE_TYPE::NONE;
        args_parser.id = -1;
        std::cout << "无输入类型设备" << std::endl;
        return args_parser;
    }
    std::string id(argv[4]);
    try{
        args_parser.id = std::stoi(id);
    }
    catch(...){
        std::cout << "id 不是可识别数字" << std::endl;
        args_parser.device_type = DEVICE_TYPE::NONE;
        args_parser.id = -1;
    }
    return args_parser;
}

int main(int argc, char* argv[]){
    ArgsParser args_parser = argsParser(argc, argv);
    if(args_parser.device_type == DEVICE_TYPE::NONE){
        return -1;
    }

    LOGINIT();
    auto packageManager = std::make_shared<PackageManager>(TRANSPORT_CONFIG_FILE_PATH);
    auto portManager = std::make_shared<PortManager>(TRANSPORT_CONFIG_FILE_PATH, packageManager);

    
}