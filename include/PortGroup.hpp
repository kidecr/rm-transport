#ifndef __PORT_GROUP__
#define __PORT_GROUP__
#include <iostream>
#include "opencv2/opencv.hpp"

#include "Port.hpp"
#include "CanPort.hpp"

class PortGroup
{
public:
    using SharedPtr = std::shared_ptr<PortGroup>;
public: 
    std::map<std::string, std::shared_ptr<Port>> m_port_table;
public:
    PortGroup() = default;
    ~PortGroup() = default;

    PortGroup(std::string config_path)
    {
        cv::FileStorage fs(config_path, cv::FileStorage::READ);
        if(fs.isOpened())
        {
            for(auto port_name_node : fs["port_list"])
            {
                std::string port_name = port_name_node;
                if(port_name.find("can") != std::string::npos)
                {
                    m_port_table[port_name] = std::make_shared<CanPort>(port_name);
                }
                else
                {
                    std::cout << "port name illegal" << std::endl;
                }
            }
        }
        else // 默认创建一个can0
        {
            m_port_table["can0"] = std::make_shared<CanPort>("can0");
        }
    }

    inline int registerPackageManagerForPort(PackageManager::SharedPtr package_manager)
    {
        if(!package_manager)
        {
            std::cout << "in function " << __FUNCTION__ <<  ": param package_manager is nullptr" << std::endl;
            return -1;
        }
        for(auto port = m_port_table.begin(); port != m_port_table.end(); ++port)
        {
            port->second->registerPackageManager(package_manager);
        }
        return 0;
    }
};

#endif // __PORT_GROUP__