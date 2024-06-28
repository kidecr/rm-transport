#include <cstdint>
#include <string>
#include <vector>
#include <iostream>
#include <regex>

#include "protocal/Protocal.hpp"
#include "impls/logger.hpp"
#include "port/SerialPort.hpp"
#include "port/CanPort.hpp"

struct ArgsParser
{
    PORT_TYPE device_type;
    std::string port_name;
    int32_t baud;
    std::vector<uint32_t> id;
};

ArgsParser argsParser(int argc, char* argv[])
{
    ArgsParser args_parser;
    if( argc <= 5 
        || std::string(argv[1]) != "--device-type"
        || std::string(argv[3]) != "--port-name")
    {
        std::cout << "输入参数错误" << std::endl;
        std::cout << "command: RecvPackage --device-type <serial|can> --port-name <port name> [--baud <baud>] [--id <id,...>]" << std::endl;
        args_parser.device_type = PORT_TYPE::NONE;
        return args_parser;
    }
    // 获取设备类型
    std::string device_type(argv[2]);
    if(device_type == "serial"){
        args_parser.device_type = PORT_TYPE::SERIAL;
        if (argc >= 7 && std::string(argv[5]) == "--baud")
        {
            args_parser.baud = std::stoi(argv[6]);
        }
        else if (argc >= 9 && std::string(argv[7]) == "--baud")
        {
            args_parser.baud = std::stoi(argv[8]);
        }
        else{
            args_parser.device_type = PORT_TYPE::NONE;
            std::cout << "无波特率输入" << std::endl;
            return args_parser;
        }
    }
    else if (device_type == "can")
        args_parser.device_type = PORT_TYPE::CAN;
    else {
        args_parser.device_type = PORT_TYPE::NONE;
        std::cout << "无输入类型设备" << std::endl;
        return args_parser;
    }
    // 获取设备名
    args_parser.port_name = std::string(argv[4]);
    // 获取id列表
    auto getIdList = [](std::string id_list_str) -> std::vector<uint32_t> {
        std::regex validIdListPattern("0x[0-9A-Fa-f]{3}(,0x[0-9A-Fa-f]{3})*");
        std::regex string2IdPattern("0x([0-9A-Fa-f]{3})");
        std::vector<uint32_t> id_list;
        if(std::regex_match(id_list_str, validIdListPattern)){
            std::smatch match;
            // 使用 std::sregex_token_iterator 遍历匹配到的所有子串
            for (std::sregex_token_iterator it(id_list_str.begin(), id_list_str.end(), string2IdPattern, 1), end; it != end; ++it) {
                // 将十六进制字符串转换为整数
                int value = std::stoi(it->str(), nullptr, 16);
                id_list.push_back(value);
            }
        }
        else{
            std::cout << "id列表语法有错误" << std::endl;
        }
        return id_list;
    };
    // 有id列表
    if (argc >= 7 && std::string(argv[5]) == "--id")
    {
        args_parser.id = getIdList(std::string(argv[6]));
    }
    if (argc >= 9 && std::string(argv[7]) == "--id"){
        args_parser.id = getIdList(std::string(argv[8]));
    }
    
    return args_parser;
}

template<typename T>
class PackageReceiverPort : public T
{
public:
    std::vector<ID> m_recv_ids;
    // for serial
    PackageReceiverPort(std::string port_name, int baud_read) : T(port_name, baud_read) {}
    // for can
    PackageReceiverPort(std::string port_name) : T(port_name) {}

    std::string printIdList()
    {
        std::stringstream ss;
        std::shared_lock<std::shared_mutex> lock(m_id_vec_mutex);
        ss << "------------------\n";
        for(auto id : m_recv_ids)
            ss << "0x" << std::hex << transport::unmask(id) << "\n";
        return ss.str();
    }
protected:
    std::shared_mutex m_id_vec_mutex;

    bool findID(ID& id){
        std::shared_lock<std::shared_mutex> lock(m_id_vec_mutex);
        for (auto current_id : m_recv_ids)
        {
            if (id == current_id)
                return true;
        }
        return false;
    }
    virtual bool recvOnePackage(ID &id, transport::Buffer &buffer) {
        
        if(!findID(id)){
            std::lock_guard<std::shared_mutex> lock(m_id_vec_mutex);
            m_recv_ids.push_back(id);
        }
        return true;
    }
};

int main(int argc, char* argv[]){
    ArgsParser args_parser = argsParser(argc, argv);
    if(args_parser.device_type == PORT_TYPE::NONE){
        return -1;
    }
    LOGINIT("RecvPackage");
    if(args_parser.device_type == PORT_TYPE::SERIAL){
        PackageReceiverPort<transport::SerialPort> serial_port(args_parser.port_name, args_parser.baud);
        while (transport::ok())
        {
            std::cout << serial_port.printIdList() << std::endl;
            usleep(1e6); // sleep 1s
        }
    }
    else if (args_parser.device_type == PORT_TYPE::CAN){
        PackageReceiverPort<transport::CanPort> can_port(args_parser.port_name);
        while (transport::ok())
        {
            std::cout << can_port.printIdList() << std::endl;
            usleep(1e6); // sleep 1s
        }
    }
}