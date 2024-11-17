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
    PORT_TYPE device_type = PORT_TYPE::NONE;
    std::string port_name;
    int32_t baud = 0;
    bool print_buffer = false;
    std::vector<uint32_t> id;
};

ArgsParser argsParser(int argc, char* argv[])
{
    ArgsParser args_parser;
    // 首先判定输入是否合理
    enum class ARGS_TYPE {
        NORMAL, EMPTY, ILLEGAL, HELP
    };
    ARGS_TYPE args_type = ARGS_TYPE::NORMAL;
    if (argc == 1) {
        args_type = ARGS_TYPE::EMPTY;
    }
    else
    {
        if (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h"){
            args_type = ARGS_TYPE::HELP;
        }
        else if (std::string(argv[1]) == "--device-type" || std::string(argv[1]) == "-d"){
            if (argc >= 4) {
                if (std::string(argv[3]) == "--port-name" || std::string(argv[3]) == "-p") {
                    args_type = ARGS_TYPE::NORMAL;
                }
                else {
                    args_type = ARGS_TYPE::ILLEGAL;
                }
            }
            else{
                args_type = ARGS_TYPE::ILLEGAL;
            }
        }
        else{
            args_type = ARGS_TYPE::ILLEGAL;
        }
    }
    // 当存在非法输入或help时，处理输出
    if (args_type != ARGS_TYPE::NORMAL) {
        if (args_type == ARGS_TYPE::EMPTY)
            std::cout << "输入参数不可以为空" << std::endl;
        else if (args_type == ARGS_TYPE::ILLEGAL)
            std::cout << "输入参数错误" << std::endl;
        std::cout << "command: RecvPackage [--help|-h] --device-type|-d <serial|can> --port-name|-p <port name> [--baud|-b <baud>] [--id|-i <id,...>] [--print-buffer|-pb]" << std::endl;
        args_parser.device_type = PORT_TYPE::NONE;
        return args_parser;
    }

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
    // 解析参数
    for (int i = 1; i < argc; ++i) {
        std::string current_argv = argv[i];
        if (current_argv == "--device-type" || current_argv == "-d") {
            if (i + 1 < argc && argv[i+1][0] != '-') {
                std::string device_type = argv[i + 1];
                if(device_type == "serial"){
                    args_parser.device_type = PORT_TYPE::SERIAL;
                }
                else if (device_type == "can")
                    args_parser.device_type = PORT_TYPE::CAN;
                else {
                    args_parser.device_type = PORT_TYPE::NONE;
                    std::cout << "非可识别输入类型设备" << std::endl;
                    return args_parser;
                }
            }
            else {
                std::cout << "cannot parser argument --device-type|-d" << std::endl;
                args_parser.device_type = PORT_TYPE::NONE;
                return args_parser;
            }
        }
        else if (current_argv == "--port-name" || current_argv == "-p") {
            if (i + 1 < argc && argv[i+1][0] != '-') {
                args_parser.port_name = argv[i + 1];
            }
            else {
                std::cout << "cannot parser argument --port-name|-p" << std::endl;
                args_parser.device_type = PORT_TYPE::NONE;
                return args_parser;
            }
        }
        else if (current_argv == "--baud" || current_argv == "-b") {
            if (i + 1 < argc && argv[i+1][0] != '-') // 后续存在参数且不为--或-
                args_parser.baud = std::stoi(argv[i + 1]);
            else{
                std::cout << "cannot parser argument --baud|b" << std::endl;
                args_parser.device_type = PORT_TYPE::NONE;
                return args_parser;
            }
        }
        else if (current_argv == "--id" || current_argv == "-i") {
            if (i + 1 < argc && argv[i + 1][0] != '-')
                args_parser.id = getIdList(std::string(argv[i + 1]));
            else {
                std::cout << "cannot parser argument --id|-i" << std::endl;
                args_parser.device_type = PORT_TYPE::NONE;
                return args_parser;
            }
        }
        else if (current_argv == "--print-buffer" || current_argv == "-pb") {
            args_parser.print_buffer = true;
        }
    }
    
    return args_parser;
}

class BasePackagePort
{
public:
    virtual std::string printIdList() = 0;
    virtual void registerIDList(std::vector<uint32_t> &id_list) = 0;
};

template<typename T>
class PackageReceiverPort : public T, public BasePackagePort
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

    void registerIDList(std::vector<uint32_t> &id_list) {}
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

template<typename T>
class PackagePrinterPort : public T, public BasePackagePort
{
public:
    std::vector<ID> m_id_list;
    // for serial
    PackagePrinterPort(std::string port_name, int baud_read) : T(port_name, baud_read) {}
    // for can
    PackagePrinterPort(std::string port_name) : T(port_name) {}

    void registerIDList(std::vector<uint32_t> &id_list) {
        std::lock_guard<std::shared_mutex> lock(m_id_vec_mutex);
        for (auto id : id_list){
            m_id_list.push_back(id);
        }
    }

    std::string printIdList()
    {
        std::stringstream ss;
        std::lock_guard<std::mutex> lock(m_msg_vec_mutex);
        ss << "------------------\n";
        for(auto &msg : m_msg_list) {
            auto [id, buffer] = msg;
            ss << "0x" << std::hex << transport::unmask(id) << ": " << buffer << "\n";
        }
        m_msg_list.clear();
        return ss.str();
    }
protected:
    std::vector<std::tuple<ID, transport::Buffer>> m_msg_list;
    std::mutex m_msg_vec_mutex;
    std::shared_mutex m_id_vec_mutex;

    bool findID(ID& id){
        std::shared_lock<std::shared_mutex> lock(m_id_vec_mutex);
        for (auto current_id : m_id_list)
        {
            if (transport::unmask(id) == current_id)
                return true;
        }
        return false;
    }
    virtual bool recvOnePackage(ID &id, transport::Buffer &buffer) {
        
        if(findID(id)){
            auto tuple = std::make_tuple(id, buffer);
            std::lock_guard<std::mutex> lock(m_msg_vec_mutex);
            m_msg_list.push_back(tuple);
            return true;
        }
        return false;
    }
};

int main(int argc, char* argv[]){
    ArgsParser args_parser = argsParser(argc, argv);
    if(args_parser.device_type == PORT_TYPE::NONE){
        return -1;
    }
    LOGINIT("RecvPackage");
    std::shared_ptr<BasePackagePort> port = nullptr;
    if (args_parser.print_buffer) {
        if(args_parser.device_type == PORT_TYPE::SERIAL){
            port = std::make_shared<PackagePrinterPort<transport::SerialPort>>(args_parser.port_name, args_parser.baud);
        }
        else if (args_parser.device_type == PORT_TYPE::CAN){
            port = std::make_shared<PackagePrinterPort<transport::CanPort>>(args_parser.port_name);
        }
        port->registerIDList(args_parser.id);
    }
    else {
        if(args_parser.device_type == PORT_TYPE::SERIAL){
            port = std::make_shared<PackageReceiverPort<transport::SerialPort>>(args_parser.port_name, args_parser.baud);
        }
        else if (args_parser.device_type == PORT_TYPE::CAN){
            port = std::make_shared<PackageReceiverPort<transport::CanPort>>(args_parser.port_name);
        }
    }
    while (transport::ok())
    {
        std::cout << port->printIdList() << std::endl;
        usleep(1e6);
    }
    
}