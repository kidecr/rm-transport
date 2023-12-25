#ifndef __NOT_USE_LOG__
#define __NOT_USE_LOG__
#endif // __NOT_USE_LOG__


#include "PackageManager.hpp"
#include "PortManager.hpp"

#include "logger.hpp"

#include "external-interface/cxxInterface.hpp"

using namespace transport;

int main(int argc, char* argv[])
{
    try{
        LOGINIT();
        auto packageManager = std::make_shared<PackageManager>(TRANSPORT_CONFIG_FILE_PATH);
        auto portManager = std::make_shared<PortManager>(TRANSPORT_CONFIG_FILE_PATH, packageManager);
        
        auto control = std::make_shared<WMJRobotControl>(packageManager);

        control->setGimbalSpeed(0.0, 0.0);
        control->StopShoot();

    }
    catch (PortException e)
    {
        std::cout << e.what() << std::endl;
    }
    transport::shutdown();
    return 0;
}