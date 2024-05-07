#include <iostream>
#include "impls/BackGround.hpp"

int main(){
    try{
        LOGINIT()
        transport::background::BackGround background(TRANSPORT_CONFIG_XML_FILE_PATH);
        for(auto &port: background.m_port_list){
            std::cout << port.toString() << std::endl;
        }
        for(auto &package : background.m_package_list){
            std::cout << package.toString() << std::endl;
        }
    }
    catch(transport::PortException e){
        std::cout << e.what() << std::endl;
    }
    return 0;
}