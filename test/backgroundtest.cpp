#include <iostream>
#include "impls/BackGround.hpp"
#include "PackageManager.hpp"
#include "PortManager.hpp"
#include "PortScheduler.hpp"

using namespace transport;

int main(){
    try{
        LOGINIT("backgroundtest");
        auto background = std::make_shared<background::BackGround>(TRANSPORT_CONFIG_XML_FILE_PATH);
        for(auto &port: background->m_port_list){
            std::cout << port.toString() << std::endl;
        }
        for(auto &package : background->m_package_list){
            std::cout << package.toString() << std::endl;
        }

        auto packageManager = std::make_shared<PackageManager>(background);
        auto portManager = std::make_shared<PortManager>(background, packageManager);
        auto portScheduler = std::make_shared<PortScheduler>(background, portManager);

        std::cout << packageManager->toString() << std::endl;
        std::cout << portManager->toString() << std::endl;

        std::cout << SERIAL_ID_JUDGE.id << std::endl;
    }
    catch(transport::PortException e){
        std::cout << e.what() << std::endl;
    }
    return 0;
}