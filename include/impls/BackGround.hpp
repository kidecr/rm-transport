#ifndef __BACKGROUND_HPP__
#define __BACKGROUND_HPP__

#include <regex>
#include <sstream>
#include <iomanip> 
#include <cstdlib>
#include <boost/property_tree/xml_parser.hpp>

#include "impls/exception.hpp"
#include "impls/logger.hpp"
#include "utils/Utility.hpp"
#include "protocal/Protocal.hpp"
#include "protocal/IDs.hpp"

namespace transport
{
namespace background
{

namespace xml = boost::property_tree;

struct Package{
    ID m_oid;  // 原始id，package id
    ID m_id;   // 包含各种信息的id: [ reserve : 32 | group id : 8 | port id : 8 | device type : 8 | package id : 16 ] 

    int32_t m_group_id;
    int32_t m_port_id;
    std::string m_port_name;
    PORT_TYPE m_port_type;
    std::string m_package_name;
    int64_t m_debug_flag;
    int64_t m_queue_size;

    static int64_t getDebugFlag(std::string debug_flags){
//      DEBUG_PRINT_ID_IF_RECEIVED = 0x01,
//      DEBUG_PRINT_BUFFER = 0x02,
//      DEBUG_PRINT_TARGET = 0x04,
        int64_t flags = 0;
        // 使用正则表达式拆分字符串
        std::regex re("[,;| ]");    // 支持使用 ‘,’ ‘;’ ‘|’ ‘ ’四种字符作为分隔符
        std::sregex_token_iterator iter(debug_flags.begin(), debug_flags.end(), re, -1);  
        std::sregex_token_iterator end;  
        std::vector<std::string> tokens(iter, end); 
        // 将字符串转int
        for(auto flag : tokens){
            if(flag == "0" | flag == ""){
                continue;
            }
            if(flag == "DEBUG_PRINT_ID_IF_RECEIVED"){
                flags = flags | 0x01;
                continue;
            }
            if(flag == "DEBUG_PRINT_BUFFER"){
                flags = flags | 0x02;
                continue;
            }
            if(flag == "DEBUG_PRINT_TARGET"){
                flags = flags | 0x04;
                continue;
            }
            LOGWARN("unknow debug flag: %s", flag.c_str());
        }
        return flags;
    }

    static ID getId(Package &package){
        ID id = 0;
        id = package.m_group_id;
        id = id << 8;
        id = id | package.m_port_id;
        id = id << 8;
        id = id | package.m_port_type;
        id = id << 16;
        id = id | package.m_oid;
        return id;
    }

    std::string toString(){
        std::stringstream ss;
        ss << "{" <<
        " m_oid:0x" << std::hex << std::setw(4) << std::setfill('0') << m_oid <<
        " m_id:0x" << std::setw(8) << std::setfill('0') << m_id << 
        " m_debug_flag:0x" << m_debug_flag << std::dec <<
        " m_group_id:" << m_group_id <<
        " m_port_id:" << m_port_id <<
        " m_port_name:" << m_port_name <<
        " m_package_name:" << m_package_name <<
        " }";
        return ss.str();
    }
};

struct Port{
    int32_t m_port_id;      // port id
    PORT_TYPE m_port_type;  // port类型
    std::string m_port_name;    // port name
    int32_t m_group_id;     // 组id
    int32_t m_baud;         // 波特率
    std::vector<Package> m_package_list;

    std::string toString(){
        std::stringstream ss;
        ss << "{" <<
        " m_port_id:" << m_port_id <<
        " m_port_name:" << m_port_name <<
        " m_port_type:" << m_port_type <<
        " m_group_id:" << m_group_id <<
        " m_baud:" << m_baud <<
        " m_package_list:[" << std::hex;
        for(auto &package : m_package_list){
            ss << "0x" << package.m_oid << " ";
        }
        ss << "] }";
        return ss.str();
    }
};

class BackGround
{
public:
/**
 * package manager: 读id list，主要为创建id package，根据port名为id添加mask，同时指定id中的设置参数
 * port manager: 读port list，主要为创建port，并读取id list，为port绑定id
 * port scheduler: 读取scheduler group，设置port的group id
 * 
 * 数据结构包含3个对外接口：遍历id的接口，为package manager创建package，其中每个package应包含所属port group和参数
 *                      遍历port接口，为port manager创建port，并可以遍历上面的结构绑定package，包含port和group数据
 *                      遍历port接口, 为port scheduler激活调度组结构
 * 
*/

    std::vector<Package> m_package_list;
    std::vector<Port> m_port_list;

    using SharedPtr = std::shared_ptr<BackGround>;

public:
    BackGround(std::string cfg_path = TRANSPORT_CONFIG_XML_FILE_PATH)
    {
        if (!isFile(cfg_path.c_str()))
            throw PORT_EXCEPTION("cfg_path is not a file!");

        xml::ptree root;
        int32_t group_id = 0;
        int32_t port_id = 0;
        try
        {
            xml::read_xml(cfg_path, root);
            xml::ptree groups = root.get_child("opencv_storage");
            for (auto &group_node : groups)
            {
                if (group_node.first != "group") // 跳过root节点可能存在的其他部分
                    continue;

                for (auto &port_node : group_node.second)
                {
                    if (port_node.first != "port") // 跳过group节点可能存在的其他部分
                        continue;
                    Port port;
                    port.m_group_id = group_id;
                    port.m_port_id = port_id;
                    port.m_port_name = port_node.second.get<std::string>("<xmlattr>.name");

                    if (port_node.second.get<std::string>("<xmlattr>.type") == "can")
                    {
                        port.m_port_type = PORT_TYPE::CAN;
                    }
                    if (port_node.second.get<std::string>("<xmlattr>.type") == "serial")
                    {
                        port.m_port_type = PORT_TYPE::SERIAL;
                        port.m_baud = std::strtol(port_node.second.get<std::string>("<xmlattr>.baud").c_str(), 0, 0);
                    }
                    for (auto &package_node : port_node.second)
                    {
                        if (package_node.first != "package") // 去掉port里的其他部分
                            continue;
                        Package package;
                        package.m_oid = std::strtol(package_node.second.get<std::string>("<xmlattr>.id").c_str(), 0, 0);
                        package.m_group_id = group_id;
                        package.m_port_id = port_id;
                        package.m_port_name = port.m_port_name;
                        package.m_port_type = port.m_port_type;
                        package.m_package_name = package_node.second.get<std::string>("<xmlattr>.name", "");
                        package.m_debug_flag = Package::getDebugFlag(package_node.second.get<std::string>("<xmlattr>.debug"));
                        package.m_queue_size = std::strtol(package_node.second.get<std::string>("<xmlattr>.queue", "1").c_str(), 0, 0);
                        package.m_id = Package::getId(package);

                        // 给PackageID全局变量赋值
                        if(!package.m_package_name.empty()){
                            SET_PACKAGE(package.m_package_name, package.m_id);
                        }

                        m_package_list.push_back(package);
                        port.m_package_list.push_back(package);
                    }
                    m_port_list.push_back(port);
                    ++port_id;
                }
                ++group_id;
            }
        }
        catch (const xml::xml_parser::xml_parser_error &e)
        {
            throw PORT_EXCEPTION(e.what());
        }
    }
};

} // namespace background
} // namespace transport
#endif // __BACKGROUND_HPP__