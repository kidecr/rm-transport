#include <iostream>
#include "impls/Config.hpp"
#include "PackageManager.hpp"
#include "PortManager.hpp"
#include "PortScheduler.hpp"

using namespace transport;

int main(){
    try{
        LOGINIT("configtest");
        auto config = std::make_shared<config::Config>(TRANSPORT_CONFIG_XML_FILE_PATH);
        for(auto &port: config->m_port_list){
            std::cout << port.toString() << std::endl;
        }
        for(auto &package : config->m_package_list){
            std::cout << package.toString() << std::endl;
        }
        std::cout << "reinit_cnt: " << config->m_reinit.m_reinit_cnt << std::endl;
        std::cout << "user passwd: " << config->m_user.passwd << std::endl;
        auto packageManager = std::make_shared<PackageManager>(config);
        auto portManager = std::make_shared<PortManager>(config, packageManager);
        auto portScheduler = std::make_shared<PortScheduler>(config, portManager);

        std::cout << packageManager->toString() << std::endl;
        std::cout << portManager->toString() << std::endl;

        std::cout << std::hex << SERIAL_ID_JUDGE.id << std::endl;
    }
    catch(transport::PortException e){
        std::cout << e.what() << std::endl;
    }
    return 0;
}