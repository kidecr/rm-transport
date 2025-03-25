/**
 * @file RecvPackage.cpp
 * @author kidecr
 * @brief 该文件主要用于检测收包与收包正确性，包括两个主要功能
 *          1. 检测特定端口都收到了什么包,并一一列出
 *             其每隔1段时间输出一次，输出这段时间内收到的包的id，该id为16位id，如0x301这种
 *             使用方法如： RecvPackage --device-type serial --port-name /dev/ttyUSB0 --baud 9600
 *          2. 检测特定端口是否收到某些包，并输出收到的包内容，可以用来查看是否收到特定包和内容
 *             其每隔1段时间输出一次，输出这段时间收到的，包含在给出的id列表内的包id和内容, 注意一定要在命令中加上--print-buffer或-pb
 *             使用方法如： RecvPackage --device-type can --port-name can0 --id 0x301,0x402 --print-buffer
 * @version 0.1
 * @date 2024-11-17
 * 
 * @copyright Copyright (c) 2024
 * 
 */ 
#include <cstdint>
#include <string>
#include <vector>
#include <iostream>
#include <regex>
#include <shared_mutex>

#include "utils/Buffer.hpp"
#include "utils/mask.hpp"
#include "utils/Utility.hpp"
#include "protocal/Protocal.hpp"
#include "impls/logger.hpp"
#include "impls/PackageID.hpp"
#include "port/SerialPort.hpp"
#include "port/CanPort.hpp"
#include "port/WinBLEPort.hpp"

struct ArgsParser
{
    PORT_TYPE device_type = PORT_TYPE::NONE;
    std::string port_name;
    int32_t baud = 0;
    bool print_buffer = false;
    std::vector<uint32_t> id;
};

static ArgsParser argsParser(int argc, char* argv[])
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
        std::regex validIdListPattern("0x[0-9A-Fa-f]{3,4}(,0x[0-9A-Fa-f]{3,4})*");
        std::regex string2IdPattern("0x([0-9A-Fa-f]{3,4})");
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
                else if (device_type == "bluetooth")
                    args_parser.device_type = PORT_TYPE::BLUETOOTH;
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
        std::lock_guard<std::shared_mutex> lock(m_id_vec_mutex);
        ss << "------------------\n";
        for(auto id : m_recv_ids)
            ss << "0x" << std::hex << transport::unmask(id) << "\n";
        m_recv_ids.clear();
        return ss.str();
    }

    void registerIDList(std::vector<uint32_t> &id_list) {}
protected:
    std::shared_mutex m_id_vec_mutex;

    bool findID(ID& id){
        std::shared_lock<std::shared_mutex> lock(m_id_vec_mutex);
        for (auto current_id : m_recv_ids)
        {
            if (transport::unmask(id) == current_id)
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
            auto& [id, buffer] = msg;
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
#ifdef ENABLE_SERIAL_PORT
        if(args_parser.device_type == PORT_TYPE::SERIAL){
            port = std::make_shared<PackagePrinterPort<transport::SerialPort>>(args_parser.port_name, args_parser.baud);
        }
        else 
#endif // ENABLE_SERIAL_PORT
#ifdef ENABLE_UNIX_CAN_PORT
        if (args_parser.device_type == PORT_TYPE::CAN){
            port = std::make_shared<PackagePrinterPort<transport::CanPort>>(args_parser.port_name);
        }
#endif // ENABLE_UNIX_CAN_PORT
#ifdef ENABLE_WIN_BLUETOOTH
        if (args_parser.device_type == PORT_TYPE::BLUETOOTH){
            port = std::make_shared<PackagePrinterPort<transport::BluetoothPort>>(args_parser.port_name);
        }
#endif // ENABLE_WIN_BLUETOOTH
        if(port){
            port->registerIDList(args_parser.id);
        }
        else {
            throw PORT_EXCEPTION("没有合适的端口实现");
        }
    }
    else {
#ifdef ENABLE_SERIAL_PORT
        if(args_parser.device_type == PORT_TYPE::SERIAL){
            port = std::make_shared<PackageReceiverPort<transport::SerialPort>>(args_parser.port_name, args_parser.baud);
        }
        else 
#endif // ENABLE_SERIAL_PORT
#ifdef ENABLE_UNIX_CAN_PORT
        if (args_parser.device_type == PORT_TYPE::CAN){
            port = std::make_shared<PackageReceiverPort<transport::CanPort>>(args_parser.port_name);
        }
#endif // ENABLE_UNIX_CAN_PORT
#ifdef ENABLE_WIN_BLUETOOTH
        if (args_parser.device_type == PORT_TYPE::BLUETOOTH){
            port = std::make_shared<PackagePrinterPort<transport::BluetoothPort>>(args_parser.port_name);
        }
#endif // ENABLE_WIN_BLUETOOTH
    }
    while (transport::ok())
    {
        std::cout << port->printIdList() << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
}