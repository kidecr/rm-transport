#include "Package.hpp"
#include "PackageManager.hpp"
#include "CanPort.hpp"
#include "GimbalPose.hpp"
#include "Shoot.hpp"
#include "PortController.hpp"


#ifndef __ROS__

#include "external-interface/cxxInterface.hpp"

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    try
    {
        auto canport = std::make_shared<CanPort>("can0");
        auto packageManager = std::make_shared<PackageManager>();
        // packageManager->add(GIMBAL);
        // packageManager->add(SHOOT);
        // packageManager->add(GYRO);
        packageManager->autoAdd("../config/PackageList.yaml");
        canport->registerPackageManager(packageManager);
        auto control = std::make_shared<WMJRobotControl>(packageManager);
        while (1)
        {
            GimbalPose pose;
            control->setGimbalPose(pose);
            usleep(1e6);
            pose = control->getGimbalPose();
            std::cout << pose.toString() << std::endl;
        }
        
    }
    catch (CanPortException &e)
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
        packageManager->add(GIMBAL);
        packageManager->add(GYRO);
        packageManager->add(SHOOT);
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
    catch (CanPortException *e)
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