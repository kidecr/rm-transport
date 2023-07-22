#include "Package.hpp"
#include "PackageManager.hpp"
#include "CanPort.hpp"
#include "GimbalPose.hpp"
#include "Shoot.hpp"
#include "PortController.hpp"
#include "PortGroup.hpp"

#ifndef __ROS__

#include "external-interface/cxxInterface.hpp"

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    try
    {
        auto portGroup = std::make_shared<PortGroup>("../config/PackageList.yaml");
        auto packageManager = std::make_shared<PackageManager>();
        auto portControler = std::make_shared<PortController>();

        packageManager->addIDFromConfigFile("../config/PackageList.yaml");
        portGroup->registerPackageManagerForPort(packageManager);
        // can1->registerPackageManager(packageManager);
        auto control = std::make_shared<WMJRobotControl>(packageManager);
        portControler->registerPortFromPortGroup(portGroup);
        portControler->run();

        int i = 0;
        while (1)
        {
            GimbalPose pose;
            pose.yaw = 10, pose.pitch = 20;
            std::cout << "---------------先发包-------------------" << std::endl;
            control->setGimbalPose(pose);
            control->switchCoor(true);
            control->setGimbalPose(pose);
            control->shootSome(++i);
            usleep(1e6);
            std::cout << "---------------后收包-------------------" << std::endl;
            std::cout << control->getGimbalPose().toString() << std::endl;
            control->switchCoor(false);
            std::cout << control->getGimbalPose().toString() << std::endl;
            std::cout << control->getShootPackage().toString() << std::endl;
        }
        
    }
    catch (PortException &e)
    {
        std::cout << e.what() << std::endl;
    }

    // node.spin(gimbal);
    return 0;
}

#else

#include "rclcpp/rclcpp.hpp"
#include "external-interface/Gimbal.hpp"
#include "external-interface/Gimbal2.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        auto canport = std::make_shared<CanPort>("can0");
        auto packageManager = std::make_shared<PackageManager>();
        auto portControler = std::make_shared<PortController>();
        packageManager->addIDFromConfigFile("../config/PackageList.yaml");
        canport->registerPackageManager(packageManager);
        portControler->registerPort(canport);
        portControler->run();

        auto node = std::make_shared<rclcpp::Node>("transport");
        auto gimbal_node = std::make_shared<Gimbal>(node, packageManager);
        auto gimbal2_node = std::make_shared<Gimbal2>(node, packageManager);

        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    }
    catch (PortException *e)
    {
        std::cout << e->what() << std::endl;
        delete e;
    }
    catch (...)
    {
        std::cout << "what error?" << std::endl;
    }

    // node.spin(gimbal);
    return 0;
}

#endif // __ROS__